#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <iostream>

using namespace Eigen;
using namespace std;
namespace py = pybind11;

class ICP {
public:
    ICP() = default;
    ~ICP() = default;

    vector<Matrix3f> H_trace;

    Matrix3f run(const std::vector<Vector2f>& target, const std::vector<Vector2f>& src, int max_iter, float tol, bool trace = false) {
        Matrix3f H;
        bool initialized = false;
        if (trace){
            H_trace.clear();
        }
        H.setConstant(std::numeric_limits<double>::quiet_NaN());
        MatrixXd goal = vectorToMatrix(target);
        MatrixXd src_mat = vectorToMatrix(src);
        MatrixXf src_sort;
        float dErr = MAXFLOAT;
        float preError = MAXFLOAT;

        for (int i = 0; i<max_iter; i++) {
            if (abs(dErr) < tol) {
                break;
            }
            float error = point_error(goal, src_mat);
            MatrixXi indexes = getNearestNeighbors(goal, src_mat).cast<int>();
            src_sort = sortPoints(src_mat.cast<float>(), indexes);

            JacobiSVD<MatrixXd> homogenous = getHomogenous(goal, src_sort.cast<double>());
            Matrix2d R = getRotation(homogenous);
            Vector2d t = getTranslation(getCenterOfMass(goal), getCenterOfMass(src_sort.cast<double>()), R);
            src_mat = src_mat * R.transpose();
            src_mat = src_mat.rowwise() + t.transpose();
            dErr = preError - error;

            if (dErr < 0 && calc_dist_means(goal, src_mat) > 10) {
                break;
            }

            preError = error;
            if (!initialized){
                H = update_homogenous(R.cast<float>(), t.cast<float>());
                initialized = true;
            } else {
                H = update_homogenous(H, R.cast<float>(), t.cast<float>());
                cout << H << endl;
            }

            if (trace){
                H_trace.push_back(H);
            }

            if (abs(dErr) < tol) {
                break;
            }
        }
        return H;
    }
private:
    static Matrix3f update_homogenous(const Matrix3f& Hin, const Matrix2f& R, const Vector2f& t) {
        Matrix3f H = Matrix3f::Zero(3, 3);
        H.block(0, 0, 2, 2) = R;
        H.block(0, 2, 2, 1) = t;
        H(2, 2) = 1;

        return H * Hin;
    }

    static Matrix3f update_homogenous(const Matrix2f& R, const Vector2f& t) {
        Matrix3f H = Matrix3f::Zero(3, 3);
        H.block(0, 0, 2, 2) = R;
        H.block(0, 2, 2, 1) = t;
        H(2, 2) = 1;

        return H;
    }

    static double point_error(const MatrixXd& goal, const MatrixXd& src) {
        double sum = 0;
        for (int i = 0; i < goal.rows(); i++) {
            sum += (goal.row(i) - src.row(i)).norm();
        }
        return sum;
    }

    MatrixXd tiled_matrix(const MatrixXd& mat, int n) {
        int rows = mat.rows();
        int cols = mat.cols();

        MatrixXd tiled_matrix(rows*n, cols);

        for (int i = 0; i < n; ++i) {
            tiled_matrix.block(i*rows, 0, rows, cols) = mat;
        }

        return tiled_matrix;
    }

    MatrixXd custom_repeat(const MatrixXd& mat, int n) {
        int rows = mat.rows();
        int cols = mat.cols();

        MatrixXd repeated_matrix(rows*n, cols);

        for (int i = 0; i < n; ++i) {
            repeated_matrix.block(i*rows, 0, rows, cols) = mat.row(i).replicate(rows, 1);;
        }

        return repeated_matrix;
    }

    VectorXd calculate_distances(const MatrixXd& current_points, const MatrixXd& previous_points) {
        VectorXd out = (custom_repeat(current_points, previous_points.rows()) - tiled_matrix(previous_points, current_points.rows())).rowwise().norm();
        return out;
    }

    VectorXd argmins(const MatrixXd& ve) {
        VectorXd indices = VectorXd::Zero(ve.rows());
        for (int i = 0; i < ve.rows(); ++i) {
            double min = ve.row(i).minCoeff();
            for (int j = 0; j < ve.cols(); ++j) {
                if (ve(i, j) == min) {
                    indices(i) = j;
                    break;
                }
            }
        }
        return indices;
    }

    VectorXd getNearestNeighbors(const MatrixXd& current_points, const MatrixXd& previous_points) {
        return argmins(calculate_distances(current_points, previous_points).reshaped(current_points.rows(), previous_points.rows()));
    }

    MatrixXd vectorToMatrix(const std::vector<Vector2f>& vec) {
        MatrixXd mat(vec.size(), 2);
        for (int i = 0; i < vec.size(); i++) {
            mat(i, 0) = vec[i](0);
            mat(i, 1) = vec[i](1);
        }
        return mat;
    }

    Matrix2d getRotation(JacobiSVD<MatrixXd> homogenous){
        MatrixXd U = homogenous.matrixU();
        MatrixXd V = homogenous.matrixV();
        Matrix2d R = V*U.transpose();
        if (R.determinant() < 0) {
            V.col(1) *= -1;
            R = V * U.transpose();
        }
        return R;
    }

    Vector2d getTranslation(Vector2d com_prev, Vector2d com_curr, Matrix2d R){
        return com_prev - R * com_curr;
    }

    JacobiSVD<MatrixXd> getHomogenous(MatrixXd prev, MatrixXd curr){
        Vector2d com_prev = getCenterOfMass(prev);
        Vector2d com_curr = getCenterOfMass(curr);
        MatrixXd prev_centered = prev.rowwise() - com_prev.transpose();
        MatrixXd curr_centered = curr.rowwise() - com_curr.transpose();

        MatrixXd W = curr_centered.transpose() * prev_centered;
        // Get Singular Value Decomposition
        JacobiSVD<MatrixXd> svd(W, ComputeThinU | ComputeThinV);
        return svd;
    }

    MatrixXf sortPoints(const MatrixXf& src, const VectorXi& indexes) {
        MatrixXf sorted(src.rows(), src.cols());
        for (int i = 0; i < src.rows(); i++) {
            sorted.row(i) = src.row(indexes(i));
        }
        return sorted;
    }

    double calc_dist_means(const MatrixXd &p1, const MatrixXd &p2) {
        return (p1.colwise().mean() - p2.colwise().mean()).norm();
    }

    Vector2d getCenterOfMass(const MatrixXd& mat) {
        Vector2d com = Vector2d::Zero();
        for (int i = 0; i < mat.rows(); i++) {
            com += mat.row(i).transpose();
        }
        return com / mat.rows();
    }

};

PYBIND11_MODULE(icp, m){
    m.doc() = "Iterative Closest Point (ICP) algorithm implemented in C++";
    py::class_<ICP>(m, "ICP")
        .def(py::init<>())
        .def("run", &ICP::run, py::arg("target"), py::arg("src"), py::arg("max_iter"), py::arg("tol"), py::arg("trace") = false)
        .def_readonly("H_trace", &ICP::H_trace);
}