from MeArm import ServoSB
from links import MultiLink
import numpy as np
import math
from frames import rotz


class MeArm(object):
    def __init__(self):

        self._L1 = 6.2  # Shoulder to elbow length
        self._L2 = 8.5  # Elbow to wrist length
        self._L3 = 8.5  # Wrist to hand length

        self._A1 = 60   # Shoulder to elbow angle
        self._A2 = -20  # Elbow to wrist angle
        self._A3 = -30  # Wrist to hand angle

        self.servos = {"base": ServoSB(7, -90, 90),
                       "shoulder": ServoSB(8, -90, 90),
                       "elbow": ServoSB(9, -90, 90),
                       "grip": ServoSB(10, 0, 90)}

        self._body = MultiLink()
        self._body.addLink(self._L1, self._A1)
        self._body.addLink(self._L2, self._A2)
        self._body.addLink(self._L3, self._A3)
        self._body.compose()


    def base(self, angle):
        print ("Moving base to:", angle)
        self.servos["base"].set_angle(angle)

    def shoulder(self, angle):
        print ("Moving shoulder to:", angle)
        self.servos["shoulder"].set_angle(angle)

    def elbow(self, angle):
        print ("Moving elbow to:", angle)
        self.servos["elbow"].set_angle(angle)

    def gripper(self, angle):
        print ("Moving gripper to:", angle)
        self.servos["grip"].set_angle(angle)

    def openGripper(self):
        print ("Grip open")
        self.gripper(90)

    def closeGripper(self):
        print ("Grip close")
        self.gripper(0)

    def clap(self):
        self.openGripper();
        self.closeGripper()
        self.openGripper();
        self.closeGripper()
        self.openGripper()

    def gotoPoint(self, x, y, z, ik_pos_threshold=1, max_tries = 20, damp_deg=math.inf):
        print ("Moving to:", (x, y, z))

        current_tries = 0
        current_effector_index = len(self._body.links)
        servos = ["base", "shoulder", "elbow", "grip"]
        servos_rotation = [self.base, self.shoulder, self.elbow, self.gripper]

        while current_tries < max_tries:

            # Extract the x, y and z coordinates of the current effector root from Hs
            # Corresponds to R in Figure 3A
            eff_root_pos_x = self._body.tt.Hs[current_effector_index - 1][3]
            eff_root_pos_y = self._body.tt.Hs[current_effector_index - 1][7]
            eff_root_pos_z = self._body.tt.Hs[current_effector_index - 1][11]


            # Extract the x, y and z coordinates of the current effector end from Hs
            # Corresponds to E in Figure 3A
            eff_end_pos_x = self._body.tt.Hs[current_effector_index][3]
            eff_end_pos_y = self._body.tt.Hs[current_effector_index][7]
            eff_end_pos_z = self._body.tt.Hs[current_effector_index][11]

            current_end = [eff_root_pos_x, eff_end_pos_y, eff_end_pos_z]
            desired_end = [x, y, z]

            # Calculate the squared distance between the current effector end and the desired position
            squared_distance = calculate_squared_distance(current_end, desired_end)
            if squared_distance <= ik_pos_threshold:
                break

            # Construct the current vector (corresponding to RE in Figure 3A)
            cur_vector_x = eff_end_pos_x - eff_root_pos_x
            cur_vector_y = eff_end_pos_y - eff_root_pos_y
            cur_vector_z = eff_end_pos_z - eff_root_pos_z
            current_vector = [cur_vector_x, cur_vector_y, cur_vector_z]

            # Construct the target vector (corresponding to RD in Figure 3A)
            target_vector_x = x - eff_root_pos_x
            target_vector_y = y - eff_root_pos_y
            target_vector_z = z - eff_root_pos_z
            target_vector = [target_vector_x, target_vector_y, target_vector_z]

            # Calculate the cos of the angle between the 2 vectors
            cos_angle = np.dot(target_vector, current_vector) / calculate_squared_distance(target_vector, current_vector)

            # Obtain the turning angle in degrees
            turn_angle = math.acos(cos_angle)
            turn_deg = math.degrees(turn_angle)

            # Perform damping
            if turn_deg > damp_deg:
                turn_deg = damp_deg

            if cos_angle < 0.99999:

                # Calculate the cross product between the target vector and the current vector to check the direction of the rotation
                cross_result = np.cross(target_vector, current_vector)
                print(cross_result, "cross_result")

                # If the z element of the cross product is positive, rotate clockwise
                if cross_result[2] > 0:

                    # Restrict rotation to the servo min angle
                    servo_min_angle = self.servos[servos[current_effector_index - 1]].minAngle
                    link_current_angle = float(self._body.links[current_effector_index - 2].angle)

                    if link_current_angle - turn_deg < servo_min_angle:
                        turn_deg = servo_min_angle

                    # Turn the link
                    servos_rotation[current_effector_index - 1](turn_deg)
                    self._body.tt.Hs[current_effector_index] = rotz(turn_deg)

                # If the z element of the cross product is negative, rotate counter-clockwise
                elif cross_result[2] < 0:

                    # Restrict rotation to the servo max angle
                    servo_max_angle = self.servos[servos[current_effector_index - 1]].maxAngle
                    link_current_angle = float(self._body.links[current_effector_index - 2].angle)

                    if link_current_angle + turn_deg > servo_max_angle:
                        turn_deg = servo_max_angle

                    # Turn the link
                    servos_rotation[current_effector_index - 1](turn_deg)
                    self._body.tt.Hs[current_effector_index] = rotz(turn_deg)

            # Update the index of the current link
            if current_effector_index == 0:
                current_effector_index = len(self._body.links)
            else:
                current_effector_index -= 1

            # Increment the number of tries
            current_tries += 1


def calculate_squared_distance(point1, point2):
    """ A function to calculate the Euclidean distance between two points """
    current_index = 0
    squared_distance = 0
    while current_index < len(point1) - 1 and current_index < len(point2) - 1:
        squared_distance += (point1[current_index] - point2[current_index])**2
        current_index += 1
    return squared_distance

if __name__ == "__main__":
    from sympy import init_printing, pprint

    init_printing(use_latex=True)
    arm = MeArm()
    print (len(arm._body.tt.Hs))
    print (len(arm._body.links))
    arm.clap()
    #Go up and left to grab something
    arm.gotoPoint(-80, 100, 140);
    arm.closeGripper();
    # Go down, forward and right to drop it
    arm.gotoPoint(70, 200, 10);
    arm.openGripper();
    # Back to start position
    arm.gotoPoint(0, 100, 50);
    print(arm._body.tt.Hs[-1])