import random
import numpy as np

arr = [random.randint(0, 100) for i in range(10)]


def isSorted(arr):
    for i in range(len(arr)-1):
        if arr[i] > arr[i+1]:
            return False
    return True


def bubbleSort(arr):
    iters = 0
    while not isSorted(arr):
        iters += 1
        for i in range(len(arr)-1):
            if arr[i] > arr[i+1]:
                arr[i], arr[i+1] = arr[i+1], arr[i]
        print(f"Iteration {iters}: {arr}")
    return arr


def insSort(arr):
    for i in range(1, len(arr)):
        key, j = arr[i], i-1
        while j >= 0 and key < arr[j]:
            arr[j+1] = arr[j]
            j -= 1
        arr[j+1] = key