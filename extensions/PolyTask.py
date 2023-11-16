import uuid
import queue
import threading
import multiprocessing


class BaseTask:
    """Base class for tasks."""
    def __init__(self, func, args, name=None):
        """
        Initialize the base task.

        :param func: Function to be executed by the task.
        :param args: Arguments to pass to the function.
        """
        self.id = str(uuid.uuid4())
        self.func = func
        self.args = args
        self.name = name
        self.cancelled = False

    def __repr__(self):
        return f"<{self.__class__.__name__} id={self.id}, name={self.name}>"

    def run(self):
        """Execute the task's function. To be overridden by subclasses."""
        pass

    def cancel(self):
        """Mark the task as cancelled."""
        self.cancelled = True


class SentinelTask:
    """A task that does nothing and is used to signal the end of tasks."""
    def run(self):
        pass


manager = multiprocessing.Manager()
ProcessReturnProxy = manager.dict


class StaticTask(BaseTask):
    """Task that executes a function without creating a new process or thread."""
    def run(self):
        """Execute the task's function."""
        if self.cancelled:
            print(f"Task {self.id} was cancelled before execution.")
            return
        self.func(*self.args)


class AsyncTask(BaseTask):
    """Task designed to run a given function in a separate process."""
    def __init__(self, func, args, manager, name=None):
        """
        Initialize the threaded task.

        :param func: Function to be executed in a separate process.
        :param args: Arguments to pass to the function.
        :param manager: TaskManager instance managing this task.
        """
        super().__init__(func, args, name)
        self.manager = manager
        self.process = None

    def run(self):
        """Start a new process to execute the task's function."""
        if self.cancelled:
            print(f"Task {self.id} was cancelled before execution.")
            return
        self.process = multiprocessing.Process(target=self.func, args=self.args)
        self.process.start()
        self.manager.futures[self.id] = self.process

    def kill(self):
        """Terminate the running process."""
        if self.process:
            self.process.terminate()


class ReturningTask(BaseTask):
    """Task that executes a function and stores its result."""
    def __init__(self, func, args, return_handle, name=None):
        """
        Initialize the returning task.

        :param func: Function whose result needs to be stored.
        :param args: Arguments to pass to the function.
        :param return_handle: An object where the result will be stored.
        """
        super().__init__(func, args, name)
        self.return_handle = return_handle

    def run(self):
        """Execute the task's function and store the result."""
        if self.cancelled:
            print(f"Task {self.id} was cancelled before execution.")
            return
        result = self.func(*self.args)
        self.return_handle[self.id] = result


class AsyncReturningTask(BaseTask):
    """Task designed to run a given function in a separate process and return a result.
    Note: Only use this if you really are storing the value. If not, use Async or returning individually."""

    def __init__(self, func, args, manager, return_proxy, name=None):
        """
        Initialize the asynchronous returning task.

        :param func: Function to be executed in a separate process and whose result needs to be stored.
        :param args: Arguments to pass to the function.
        :param manager: TaskManager instance managing this task.
        :param return_proxy: ProcessReturnProxy where the result will be stored.
        """
        super().__init__(func, args, name)
        self.manager = manager
        self.return_proxy = return_proxy
        self.process = None

    def run(self):
        """Start a new process to execute the task's function and store the result."""
        if self.cancelled:
            print(f"Task {self.id} was cancelled before execution.")
            return

        def target_func():
            """Wrapper function to capture the result and store it in the proxy."""
            result = self.func(*self.args)
            self.return_proxy[self.id] = result

        self.process = multiprocessing.Process(target=target_func)
        self.process.start()
        self.manager.futures[self.id] = self.process

    def kill(self):
        """Terminate the running process."""
        if self.process:
            self.process.terminate()


class TaskManager:
    """Manages tasks, ensuring they are executed and providing management functionalities."""
    def __init__(self, die_with_main=True):
        """Initialize the task manager.
        Note: If setting die_with_main=False, TaskManager.stop() must be called where TaskManager was initialized."""
        self.taskQueue = queue.Queue()
        self.inputQueue = queue.Queue()  # New input queue for tasks
        self.futures = {}
        self.stop_flag = False

        self.dispatcherThread = threading.Thread(target=self.task_dispatcher, daemon=True)
        self.dispatcherThread.start()

        self.masterTask = threading.Thread(target=self.monitor_tasks, daemon=die_with_main)
        self.masterTask.start()

    def task_dispatcher(self):
        """Dispatch tasks from inputQueue to taskQueue."""
        while not self.stop_flag:
            try:
                task = self.inputQueue.get(block=False)
                if isinstance(task, SentinelTask):  # Check for sentinel task
                    print("Ending manager, sentinel received.")
                    self.taskQueue.put(StaticTask(lambda: 2+2, ()))
                    return
                self.taskQueue.put_nowait(task)
            except queue.Empty:
                pass

    def monitor_tasks(self):
        """Monitor and execute tasks added to the task manager."""
        while not self.stop_flag:
            try:
                current_task = self.taskQueue.get(block=False)
                print(f"Running {current_task}")
                current_task.run()
                print(f"Ran {current_task}")
            except queue.Empty:
                print("Got no tasks. Waiting a second....")
                time.sleep(1)

    def add_task(self, func, args=(), type="static"):
        """
        Add a new task to the manager.

        :param func: Function to be executed by the task.
        :param args: Arguments to pass to the function.
        :param type: Type of the task. Can be "static", "async", or "returning".
        :return: ID of the added task. If the task returns, also return a PRP handle.
        """
        cache = None
        if type == "static":
            task = StaticTask(func, args)
        elif type == "async":
            task = AsyncTask(func, args, self)
        elif type == "returning":
            cache = ProcessReturnProxy()
            task = ReturningTask(func, args, cache)
        elif type == "async-returning":
            cache = ProcessReturnProxy()
            task = AsyncReturningTask(func, args, self, cache)
        else:
            raise ValueError("Unknown task type")
        self.inputQueue.put_nowait(task)
        if cache is not None:
            return task.id, cache
        else:
            return task.id

    def stop_task(self, task_id):
        """
        Stops a task. If the task is in the queue and hasn't been processed yet, it will be cancelled.
        If the task is already running asynchronously, it will be terminated.
        There is a chance of a race condition happening.

        :param task_id: ID of the task to be stopped.
        :return: True if the task was stopped or cancelled, False otherwise.
        """

        # Check if the task is in the queue
        for task in list(self.taskQueue.queue):
            if task.id == task_id:
                task.cancel()
                return True

        # If not in the queue, try to terminate the async task
        process = self.futures.get(task_id)
        if process:
            process.terminate()
            del self.futures[task_id]
            return True

        return False

    def print_active_tasks(self):
        """Print details of all active tasks."""
        print("\nActive Tasks:")
        # Print tasks in the queue
        for task in list(self.taskQueue.queue):
            print(task)
        # Print asynchronous tasks
        for task_id, process in self.futures.items():
            print(f"<AsyncTask id={task_id}, process_pid={process.pid}>")

    def wait_for_queue(self):
        """Stalls the process that this method is called on until all sequential tasks are done."""
        while self.taskQueue.qsize() > 0:
            pass

    def wait_for_async_tasks(self, ignore=None):
        """Wait for all active asynchronous tasks to complete, excluding those with IDs in ignore_ids.
        It won't find any async tasks if they haven't been started, so make sure they are started beforehand.
        A way to do this is with wait_for_queue, assuming you have only called async methods to start.
        :param ignore: List of task IDs to ignore (not wait for).
        """
        if ignore is None:
            ignore = []
        for task_id in list(self.futures.keys()):  # Create a copy of the keys
            if task_id in ignore:
                continue
            process = self.futures.get(task_id)
            if process:
                process.join()  # Wait for the process to finish

    def stop(self):
        """Stops the masterTask on the next iteration.
        If there are no active computations, add a simple one to let the thread check for the stop flag."""
        self.stop_flag = True
        self.inputQueue.put(SentinelTask())
        self.dispatcherThread.join()
        self.masterTask.join()


if __name__ == "__main__":
    import cv2
    import numpy as np
    import time

    manager = TaskManager(True)


    def bgTask():
        # Just shows a random black and white image
        while True:
            rgb = np.random.randint(2, size=(400, 400), dtype=np.uint8) * 255
            cv2.imshow("randimg", cv2.merge([rgb, rgb, rgb]))
            cv2.waitKey(1)


    def computation():
        # Takes a while to calculate.
        np.math.factorial(1000000)
        return 1


    def test(tasktype="async", amount=10):
        # This tests queueing times and sets a bunch of computations
        caches = []  # stores the return dicts. THESE MUST ALWAYS BE SAVED.
        times = 0  # Total amount of time it took to process all 10 task assignments.
        for i in range(amount):
            start = time.time()
            if tasktype == "async" or tasktype == "static":  # Non-returning tasks
                id = manager.add_task(computation, type=tasktype)
                caches.append(id)  # These don't return, so we store ids
            else:  # If it has to return a value.
                id, cache = manager.add_task(computation, type=tasktype)
                caches.append(cache)  # Store the return dicts.
            times += time.time() - start

        print(f"Assignment time: {times}")
        return caches


    bgID = manager.add_task(bgTask, type="async")  # Testing for a permanent background task

    start = time.time()

    a = test("static")  # test a certain type of task.
    print("All Tasks pushed.")
    manager.wait_for_queue()  # Wait for the task queue to clear up. If it's async, it waits for all processes to start
    manager.wait_for_async_tasks(
        [bgID])  # This waits for async tasks to finish, except for the permanent background task.
    print(f"Time: {time.time() - start}")  # Processing time
    manager.stop()  # Stop the masterTask
    manager.stop_task(bgID)  # Stop the permanent background task.
