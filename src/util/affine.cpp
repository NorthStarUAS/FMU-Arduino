#include <Arduino.h>

#include "affine.h"

static void print_matrix(const char *label, Eigen::MatrixXf M) {
    printf("%s:\n", label);
    for ( int i = 0; i < M.rows(); i++ ) {
        printf("  ");
        for ( int j = 0; j < M.cols(); j++ ) {
            if ( M(i,j) >= 0 ) {
                printf(" ");
            }
            printf("%.6f ", M(i,j));
        }
        printf("\n");
    }
    delay(100);
}

static void print_numpy_style(const char *label, Eigen::MatrixXf M) {
    printf("%s = [ ", label);
    for ( int j = 0; j < M.cols(); j++ ) {
        if ( j > 0 ) {
            printf(", ");
        }
        printf("[ ");
        for ( int i = 0; i < M.rows(); i++ ) {
            if ( i > 0 ) {
                printf(", ");
            }
            printf("%.6f", M(i,j));
        }
        printf("]");
    }
    printf("]\n");
    delay(100);
}

static void print_vector(const char *label, Eigen::VectorXf v) {
    printf("%s: ", label);
    for ( int i = 0; i < v.size(); i++ ) {
        printf("%.2f ", v(i));
    }
    printf("\n");
}

bool affine_from_points(Eigen::MatrixXf src, Eigen::MatrixXf dst, bool shear, bool scale, Eigen::MatrixXf &M ) {
    printf("affine_from_points()\n");
    int ndims = src.cols();
    if ( src.rows() != dst.rows() or src.cols() != dst.cols() ) {
        printf("src and dst are different dimensions\n");
        return false;
    }
    if ( ndims < 2 or src.cols() < ndims ) {
        printf("need at least 2 dimensions.\n");
        return false;
    }

    print_numpy_style("v0", src);
    print_numpy_style("v1", dst);

    // move src centroid to origin
    Eigen::VectorXf t0; t0.resize(ndims);
    for ( int i = 0; i < ndims; i++ ) {
        t0(i) = -src.col(i).mean();
    }
    print_vector("t0", t0);
    Eigen::MatrixXf M0;
    M0.resize(ndims+1, ndims+1);
    M0.setIdentity();
    for ( int i = 0; i < ndims; i++ ) {
        M0(i, ndims) = t0(i);
    }
    print_matrix("M0", M0);
    for ( int i = 0; i < src.rows(); i++ ) {
        src.row(i) += t0;
    }
    print_matrix("v0", src);

    // move dst centroid to origin
    Eigen::VectorXf t1; t1.resize(ndims);
    for ( int i = 0; i < ndims; i++ ) {
        t1(i) = -dst.col(i).mean();
    }
    print_vector("t1", t1);
    Eigen::MatrixXf M1;
    M1.resize(ndims+1, ndims+1);
    M1.setIdentity();
    for ( int i = 0; i < ndims; i++ ) {
        M1(i, ndims) = t1(i);
    }
    print_matrix("M1", M1);
    for ( int i = 0; i < dst.rows(); i++ ) {
        dst.row(i) += t1;
    }
    print_matrix("v1", dst);

    if ( shear ) {
        Eigen::MatrixXf A;
        A.resize(src.rows(), src.cols()*2);
        print_matrix("A.block()", A.block(0,0,src.rows(),src.cols()));
        A.block(0,0,src.rows(),src.cols()) = src;
        A.block(0,src.cols(),src.rows(),src.cols()) = dst;
        print_matrix("A", A);
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXf vh = svd.matrixV().transpose();
        print_matrix("vh orig", vh);
        Eigen::MatrixXf vht = vh.transpose().block(0,0,vh.cols(),ndims);
        print_matrix("vht", vht);
        Eigen::MatrixXf B = vht.block(0,0,ndims,ndims);
        print_matrix("B", B);
        Eigen::MatrixXf C = vht.block(ndims,0,ndims,ndims);
        print_matrix("C", C);
        Eigen::MatrixXf t = C * B.inverse();
        print_matrix("t", t);
        M.resize(ndims+1, ndims+1);
        M.setIdentity();
        M.block(0,0,ndims,ndims) = t;
        print_matrix("M", M);
    } else {
        Eigen::MatrixXf tmp = dst.transpose() * src;
        print_matrix("dot", tmp);
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(tmp, Eigen::ComputeThinU | Eigen::ComputeThinV);
        print_matrix("U", svd.matrixU());
        print_vector("s", svd.singularValues());
        print_matrix("V", svd.matrixV().transpose());

        Eigen::MatrixXf R = svd.matrixU() * svd.matrixV().transpose();
        print_matrix("R", R);
        printf("det(R): %.8f\n", R.determinant());
        if ( R.determinant() < 0.0 ) {
            printf("Not a right handed system, fixme!\n");
            return false;
        }

        M.resize(ndims+1, ndims+1);
        M.setIdentity();
        M.block(0,0,ndims,ndims) = R;
        print_matrix("M", M);
    }

    if ( scale and !shear ) {
        src = src.cwiseProduct(src);
        dst = dst.cwiseProduct(dst);
        print_matrix("v0", src);
        print_matrix("v1", dst);
        M.block(0,0,ndims,ndims) *= sqrt( dst.sum() / src.sum() );
        print_matrix("M(scale)", M);
    }

    // move centroid back
    print_matrix("M*M0", (M*M0));
    print_matrix("inv(M1)", M1.inverse());
    M = M1.inverse() * M * M0;
    print_matrix("M", M);
    printf("M[ndims,ndims]: %.4f\n", M(ndims,ndims));
    M /= M(ndims,ndims);
    print_matrix("M(final)", M);
    return true;
}

#if 0
int main() {
    Eigen::MatrixXf meas;
    // meas.resize(4,2);
    // meas <<
    //     0, 0,
    //     1031, 0,
    //     1031, 1600,
    //     0, 1600;
    meas.resize(6,3);
    meas <<
        -0.16, 0.09, -9.77,
        0.05, 0.18, 9.91,
        9.70, -0.06, 0.23,
        -9.91, 0.45, 0.08,
        0.21, 10.72, 0.31,
        -0.29, -10.44, 0.22;
    printf("[");
    for (int i = 0; i < meas.cols(); i++ ){
        printf("[");
        for (int j = 0; j < meas.rows(); j++) {
            printf("%.4f, ", meas(j, i));
        }
        printf("], ");
    }
    printf("]");
    Eigen::MatrixXf ref;
    // ref.resize(4,2);
    // ref <<
    //     675, 55,
    //     826, 52,
    //     826, 281,
    //     677, 277;
    ref.resize(6, 3);
    const float g = 9.81;
    ref <<
        0.0, 0.0,  -g,          // level
        0.0, 0.0,   g,          // upside down
         -g, 0.0, 0.0,          // nose up
          g, 0.0, 0.0,          // nose down
        0.0,  -g, 0.0,          // right wing down
        0.0,   g, 0.0;          // right wing up
    printf("[");
    for (int i = 0; i < ref.cols(); i++ ){
        printf("[");
        for (int j = 0; j < ref.rows(); j++) {
            printf("%.4f, ", ref(j, i));
        }
        printf("], ");
    }
    printf("]\n");

    Eigen::MatrixXf affine;
    affine_from_points(meas, ref, true, true, affine);
    print_matrix("returned affine", affine);
    Eigen::Matrix3f linear = affine.block(0,0,3,3);
    print_matrix("linear", linear);
    for ( int i = 0; i < 3; i++ ) {
        printf("scale(%d) = %.5f\n", i, linear.row(i).norm());
    }

    Eigen::VectorXf errors;
    errors.resize(meas.rows());
    for ( int i = 0; i < meas.rows(); i++ ) {
        Eigen::Vector4f v1; v1 << meas(i,0), meas(i,1), meas(i,2), 1.0;
        Eigen::Vector4f v2 = affine * v1;
        Eigen::Vector3f v3 = ref.row(i);
        print_vector("v1", v1);
        print_vector("v2", v2);
        print_vector("v3", v3);
        errors[i] = (v2.head(3) - v3).norm();
        printf("error[%d] = %.2f\n", i, errors[i]);
    }
    exit(0);

    Eigen::Affine3f a(Eigen::Translation3f(1,2,3));
    print_matrix("a", a.matrix());

    affine <<
        -0.97,  0.02,  0.00, -0.06,
        -0.02, -0.97,  0.00,  0.15,
        0.00,  0.00,  0.97, -0.16,
        0.00,  0.00,  0.00,  1.00;

    Eigen::Affine3f b(Eigen::Translation3f(affine.col(3).head(3)));
    print_matrix("b", b.matrix());

    Eigen::Affine3f a3f;
    a3f.matrix() = affine;

    print_matrix("Affine:", affine);
    print_matrix("Rotation:", a3f.rotation());
    print_matrix("Translation:", a3f.translation());
    print_matrix("Scale:", a3f.linear());
}
#endif
