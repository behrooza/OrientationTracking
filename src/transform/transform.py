import numpy as np



class transform:
    
    def __init__(self):
        self.q_initial = np.array([1,0,0,0])
        
    def rotation_quaternion(self, theta, v):
    
        qw = np.cos(theta/2)
        qx = v[0] * np.sin(theta/2)
        qy = v[1] * np.sin(theta/2)
        qz = v[2] * np.sin(theta/2)

        return np.array([qw, qx, qy, qz])

    def conjugate_quaternion(self, q):

        return np.array([q[0], -q[1], -q[2], -q[3]])

    def inverse_quaternion(self, q):

        q_inverse = self.conjugate_quaternion(q)/(np.square(q[0]) + np.square(q[1]) +
                                                  np.square(q[2]) + np.square(q[3]))

        return q_inverse

    def add_quaternions(self, q, p):

        return np.array([q[0] + p[0], q[1] + p[1], q[2] + p[2], q[3] + p[3]])

    def multiply_quaternions(self, q, p):

        mw = q[0]*p[0] - q[1]*p[1] - q[2]*p[2] - q[3]*p[3]
        mx = q[0]*p[1] + q[1]*p[0] + q[2]*p[3] - q[3]*p[2]
        my = q[0]*p[2] - q[1]*p[3] + q[2]*p[0] + q[3]*p[1]
        mz = q[0]*p[3] + q[1]*p[2] - q[2]*p[1] + q[3]*p[0]

        return np.array([mw, mx, my, mz])

    def rotation_matrix_to_quaternion(self, R):

        qw = np.sqrt(1 + R[0,0] + R[1,1] + R[2,2])/2
        qx = (R[2,1] - R[1,2])/(4*qw) 
        qy = (R[0,2] - R[2,0])/(4*qw)
        qz = (R[1,0] - R[0,1])/(4*qw)

        return np.array([qw, qx, qy, qz])

    def vector_quaternion(self, u):

        return np.array([0, u[0], u[1], u[2]])
    
    def initial_quaternion(self, acc):
    
        X = acc[0]
        Y = acc[1]
        Z = acc[2]

        Vi = [X, Y, Z]
        Vf = [0, 9.81, 0]

        A = np.cross(Vi, Vf)
        A_magnitude = np.linalg.norm(A)
        A_norm = A/A_magnitude
        alpha = np.arccos(Vi[1]/np.linalg.norm(Vi))

        q0 = np.cos(alpha/2)
        q1 = np.sin(alpha/2) * A_norm[0]
        q2 = np.sin(alpha/2) * A_norm[1]
        q3 = np.sin(alpha/2) * A_norm[2]

        return np.array([q0,q1,q2,q3])
    
    
    def hard_iron(self, init_hard):
        offset_x = (max(init_hard[:,0]) + min(init_hard[:,0])) / 2
        offset_y = (max(init_hard[:,1]) + min(init_hard[:,1])) / 2
        offset_z = (max(init_hard[:,2]) + min(init_hard[:,2])) / 2

        offset_hard = np.array([offset_x, offset_y, offset_z])

        return offset_hard

    def soft_iron(self, init_soft, offset_hard):

        h_cal = init_soft - offset_hard

        avg_delta_x = (max(h_cal[:,0]) - min(h_cal[:,0])) / 2
        avg_delta_y = (max(h_cal[:,1]) - min(h_cal[:,1])) / 2
        avg_delta_z = (max(h_cal[:,2]) - min(h_cal[:,2])) / 2

        avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3

        scale_x = avg_delta / avg_delta_x
        scale_y = avg_delta / avg_delta_y
        scale_z = avg_delta / avg_delta_z

        scale_soft = np.array([scale_x, scale_y, scale_z])

        return scale_soft
    
    def magnetometer_calibration(self, mag_reading, offset_hard, scale_soft):
    
        # Hard Iron
        H_mag = mag_reading - offset_hard

        # Soft Iron

        calibrated_mag = H_mag * scale_soft

        return calibrated_mag
    
    def tilt_correction(self, acc_, gyr_, q_, delta_t, alpha_t):

        if np.linalg.norm(gyr_) == 0:
            v = np.array([0,0,0])
        else:
            v = gyr_/np.linalg.norm(gyr_)   
        #v = gyr_/np.linalg.norm(gyr_) # rotation axis



        theta = np.linalg.norm(gyr_) * delta_t # amount of rotation during delta_t
        delta_q = self.rotation_quaternion(theta, v) # orientation change over time delta_t



        q_est = self.multiply_quaternions(q_, delta_q) # estimated orientation after obtaining the gyro output



        q_a_body = self.vector_quaternion(acc_) # acceleration in vector quaternion form


        acc_up = self.multiply_quaternions(self.multiply_quaternions(q_est, q_a_body),
                                           self.inverse_quaternion(q_est)) # up vector, 
        #which is first rotation of accelerometer to be in the world reference where gravity points up


        acc_up_norm = acc_up[1:]/np.linalg.norm(acc_up[1:])
        t_vector = np.array([-acc_up_norm[2], 0, acc_up_norm[0]])   
        # acc_up is a quaternion vector which contain acc values in [1:3]

        #t_vector = np.array([-acc_up_norm[2], 0, acc_up_norm[0]]) 
        # projection of acc_up (up vector) on XZ plane in the world reference

        phi = np.arccos(acc_up_norm[1]) 
        # tilt error or the angle between the up vector and the gravity vector
        if np.linalg.norm(t_vector) == 0:
            q_c_tilt = q_est
        else:
            q_c_tilt = self.multiply_quaternions(self.rotation_quaternion((1-alpha_t) * phi, t_vector/np.linalg.norm(t_vector)),
                                        q_est) # complementary filter to correct tilt error

        return q_c_tilt

    def yaw_correction(self, mag, q_tilt, mag_ref, q_ref, alpha_y):

        mag_ = self.multiply_quaternions(self.multiply_quaternions(q_tilt, self.vector_quaternion(mag)),
                                         self.inverse_quaternion(q_tilt))

        mag_ref_ = self.multiply_quaternions(self.multiply_quaternions(q_ref, self.vector_quaternion(mag_ref)),
                                             self.inverse_quaternion(q_ref))

        # rotating mag recordings from body frame to the world frame
        # mag_ is a quaternion [w, x, y, z]; heading is measured in the world XZ plane
        theta = np.arctan2(mag_[1], mag_[3])
        theta_ref = np.arctan2(mag_ref_[1], mag_ref_[3])

        q_c_yaw = self.multiply_quaternions(self.rotation_quaternion((1-alpha_y)*(theta_ref-theta), [0,1,0]), q_tilt)

        return q_c_yaw
    
    def transformation_v1(self, acc, gyr, timestamp, alpha_t):
    
        delta_t = timestamp.diff()
        delta_t[0] = 0
        delta_t = delta_t/1000

        #q_initial = np.array([1,0,0,0])#initial_quaternion(acc[0])#
        acc_tilt = np.empty(shape=(acc.shape[0],4))
        gyr_tilt = np.empty(shape=(acc.shape[0],4))
        
        #calibrated_mag = magnetometer_calibration_old(mag) # magnetometer calibration Hard Iron and Soft Iron

        q_ref = self.tilt_correction(acc[0], gyr[0], self.q_initial, delta_t[0], alpha_t)
        q_tilt = self.q_initial
        for i in range(acc.shape[0]):

            q_tilt = self.tilt_correction(acc[i], gyr[i], q_tilt, delta_t[i], alpha_t)

            acc_tilt[i] = self.multiply_quaternions(self.multiply_quaternions(q_tilt, self.vector_quaternion(acc[i])), 
                                                    self.inverse_quaternion(q_tilt))

            gyr_tilt[i] = self.multiply_quaternions(self.multiply_quaternions(q_tilt, self.vector_quaternion(gyr[i])), 
                                                     self.inverse_quaternion(q_tilt))

            self.q_initial = q_tilt

        return acc_tilt, gyr_tilt
    
    def transformation_v2(self, acc, gyr, mag, timestamp, alpha_t, alpha_y):
    
        delta_t = timestamp.diff()
        delta_t[0] = 0
        delta_t = delta_t/1000

        #q_initial = np.array([1,0,0,0])#initial_quaternion(acc[0])#
        acc_tilt = np.empty(shape=(acc.shape[0],4))
        acc_world = np.empty(shape=(acc.shape[0],4))
        gyr_world = np.empty(shape=(acc.shape[0],4))
        mag_world = np.empty(shape=(acc.shape[0],4))

        #calibrated_mag = magnetometer_calibration_old(mag) # magnetometer calibration Hard Iron and Soft Iron

        mag_ref = mag[0]
        q_ref = self.tilt_correction(acc[0], gyr[0], self.q_initial, delta_t[0], alpha_t)
        q_tilt = self.q_initial
        for i in range(acc.shape[0]):

            q_tilt = self.tilt_correction(acc[i], gyr[i], q_tilt, delta_t[i], alpha_t)

            acc_tilt[i] = self.multiply_quaternions(self.multiply_quaternions(q_tilt, self.vector_quaternion(acc[i])), 
                                                    self.inverse_quaternion(q_tilt))

            q_yaw = self.yaw_correction(mag[i], q_tilt, mag_ref, q_ref, alpha_y)

            acc_world[i] = self.multiply_quaternions(self.multiply_quaternions(q_yaw, self.vector_quaternion(acc[i])),
                                                     self.inverse_quaternion(q_yaw))

            gyr_world[i] = self.multiply_quaternions(self.multiply_quaternions(q_yaw, self.vector_quaternion(gyr[i])), 
                                                     self.inverse_quaternion(q_yaw))

            mag_world[i] = self.multiply_quaternions(self.multiply_quaternions(q_yaw, self.vector_quaternion(mag[i])), 
                                                     self.inverse_quaternion(q_yaw))
            self.q_initial = q_yaw

        return acc_tilt, acc_world, gyr_world, mag_world