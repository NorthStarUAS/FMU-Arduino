#include <Arduino.h>

#include "../nodes.h"
#include "../util/affine.h"
#include "../util/constants.h"

#include "calib_accels.h"

int calib_accels_t::raw_up_axis( float ax, float ay, float az ) {
    const float thresh = g * 0.85;
    if ( ax > thresh ) { return 0; }        // x positive, nose up
    else if ( ax < -thresh ) { return 1; }  // x negative, nose down
    else if ( ay > thresh ) { return 2; }   // y positive, right wing down
    else if ( ay < -thresh ) { return 3; }  // y negative, right wing up
    else if ( az > thresh ) { return 4; ; } // z positive, up side down
    else if ( az < -thresh ) { return 5; }  // z negative, level
    else { return -1; }                     // no axis clearly up
}

void calib_accels_t::init() {
    state = 0;                  // active
    armed = false;

    Ref.resize(6, 3);
    Ref <<
        0.0, 0.0,  -g,          // level
        0.0, 0.0,   g,          // upside down
         -g, 0.0, 0.0,          // nose up
          g, 0.0, 0.0,          // nose down
        0.0,  -g, 0.0,          // right wing down
        0.0,   g, 0.0;          // right wing up

    Meas.resize(6, 3);
    Meas.setZero();
}

void calib_accels_t::update()  {
    if ( state < 0 ) {
        return;
    }

    float dt = 0.0;
    if ( last_millis > 0 ) {
        dt = (millis() - last_millis) / 1000.0;
    }
    last_millis = millis();

    float ax = imu_node.getDouble("ax_raw");
    float ay = imu_node.getDouble("ay_raw");
    float az = imu_node.getDouble("az_raw");
    ax_slow.update(ax, dt);
    ax_fast.update(ax, dt);
    ay_slow.update(ay, dt);
    ay_fast.update(ay, dt);
    az_slow.update(az, dt);
    az_fast.update(az, dt);

    bool stable = false;
    float ax_diff = ax_slow.get_value() - ax_fast.get_value();
    float ay_diff = ay_slow.get_value() - ay_fast.get_value();
    float az_diff = az_slow.get_value() - az_fast.get_value();
    Eigen::Vector3f diff3(ax_diff, ay_diff, az_diff);
    float d = diff3.norm();
    if ( d < 0.04 ) {
        stable = true;
    }

    int up_axis = raw_up_axis(ax, ay, az);

    if ( up_axis < 0 ) {
        armed = true;
    }

    if ( state < 6 ) {
        Serial.print("up axis: "); Serial.print(up_axis);
        Serial.print(" armed: "); Serial.print(armed);
        Serial.print(" slow-fast: "); Serial.print(d, 3);
        Serial.print(" stable: "); Serial.println(stable);
    }

    if ( state == 0 ) {
        printf("Place level and right side up - stable: %d\n", stable);
        if ( armed and stable and new_axis(up_axis) ) {
            Meas(state, 0) = ax_fast.get_value();
            Meas(state, 1) = ay_fast.get_value();
            Meas(state, 2) = az_fast.get_value();
            checked[up_axis] = true;
            state += 1;
            armed = false;
        }
    } else if ( state == 1 ) {
        printf("Place upside down - stable: %d\n", stable);
        if ( armed and stable and new_axis(up_axis) ) {
            Meas(state, 0) = ax_fast.get_value();
            Meas(state, 1) = ay_fast.get_value();
            Meas(state, 2) = az_fast.get_value();
            checked[up_axis] = true;
            state += 1;
            armed = false;
        }
    } else if ( state == 2 ) {
        printf("Place nose down - stable: %d\n", stable);
        if ( armed and stable and new_axis(up_axis) ) {
            Meas(state, 0) = ax_fast.get_value();
            Meas(state, 1) = ay_fast.get_value();
            Meas(state, 2) = az_fast.get_value();
            checked[up_axis] = true;
            state += 1;
            armed = false;
        }
    } else if ( state == 3 ) {
        printf("Place nose up - stable: %d\n", stable);
        if ( armed and stable and new_axis(up_axis) ) {
            Meas(state, 0) = ax_fast.get_value();
            Meas(state, 1) = ay_fast.get_value();
            Meas(state, 2) = az_fast.get_value();
            checked[up_axis] = true;
            state += 1;
            armed = false;
        }
    } else if ( state == 4 ) {
        printf("Place right wing down - stable: %d\n", stable);
        if ( armed and stable and new_axis(up_axis) ) {
            Meas(state, 0) = ax_fast.get_value();
            Meas(state, 1) = ay_fast.get_value();
            Meas(state, 2) = az_fast.get_value();
            checked[up_axis] = true;
            state += 1;
            armed = false;
        }
    } else if ( state == 5 ) {
        printf("Place right wing up - stable: %d\n", stable);
        if ( armed and stable and new_axis(up_axis) ) {
            Meas(state, 0) = ax_fast.get_value();
            Meas(state, 1) = ay_fast.get_value();
            Meas(state, 2) = az_fast.get_value();
            checked[up_axis] = true;
            state += 1;
            armed = false;
        }
     } else if ( state == 6 ) {
        for ( int i = 0; i < 6; i++ ) {
            if ( !checked[i] ) {
                printf("Not all 6 axis calibrated, failed.\n");
                state = -1;
                return;
            }
        }
        Eigen::MatrixXf affine;
        if ( ! affine_from_points(Meas, Ref, true, true, affine) ) {
            printf("Affine fit failed...\n");
            state = -1;
            return;
        }
        for ( int j = 0; j < 4; j++ ) {
            for ( int i = 0; i < 4; i++ ) {
                imu_calib_node.setDouble("accel_affine", affine(i,j), i*4 + j);
            }
        }
        float fit = fit_metrics(affine);
        imu_calib_node.setDouble("accel_fit_mean", fit);
        Serial.print("Fit quality: "); Serial.println(fit, 2);

        // extract strapdown rotation matrix
        Eigen::Affine3f a3f;
        a3f.matrix() = affine;
        Eigen::Matrix3f strapdown = a3f.rotation();
        for ( int j = 0; j < 3; j++ ) {
            for ( int i = 0; i < 3; i++ ) {
                imu_calib_node.setDouble("strapdown", strapdown(i,j), i*3 + j);
            }
        }
        imu_calib_node.pretty_print();
        const char *file_path = "/imu-calibration.json";
        printf("Saving calibration to: %s\n", file_path);
        imu_calib_node.save(file_path);

        printf("If calibration was successful you should reboot or powercycle to get the new config.\n");
        state += 1;
    }
}

float calib_accels_t::fit_metrics(Eigen::MatrixXf affine) {
    Eigen::VectorXf errors;
    errors.resize(6);
    print_matrix("Meas", Meas);
    for ( int i = 0; i < Meas.rows(); i++ ) {
        Eigen::Vector4f v1; v1 << Meas(i,0), Meas(i,1), Meas(i,2), 1.0;
        Eigen::Vector4f v2 = affine * v1;
        Eigen::Vector3f v3 = Ref.row(i);
        print_vector("v1", v1);
        print_vector("v2", v2);
        print_vector("v3", v3);
        errors[i] = (v2.head(3) - v3).norm();
        Serial.print("error["); Serial.print(i); Serial.print(")] = "); Serial.println(errors[i], 2);
    }
    return errors.mean();
}
