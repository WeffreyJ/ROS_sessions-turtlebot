#!/usr/bin/env python3



class myTurtle():
    
    
    def __init__(self) -> None:
        """_summary_
        
        create all the nessary pubs/subs here and all the nessary other things
        
        """
        pass 
    
        

    def nav_to_pose(self, goal):
        # type: (PoseStamped) -> None
        """
        This is a callback function. It should extract data from goal, drive in a striaght line to reach the goal and
        then spin to match the goal orientation.
        :param goal: PoseStamped
        :return:
        """
        pass

    def odom_cb(self,msg:Odometry) ->None:
        """_summary_

        Get the odom and update the internal location of the robot
        Args:
            msg (Odometry): _description_
        """
        pass
    
    def stop(self)->None:
        """_summary_
        
        Stop moving
        """
        pass
        
        
    def drive_straight(self, dist: float, vel: float)->None:
        """_summary_

        Args:
            dist (_type_): _description_
        """
        pass
        
    
    def spin_wheels(self, u1, u2, time):
        """
        Spin the two wheels

        :param u1: wheel 1 speed
        :param u2: wheel 2 speed
        :param time: time to drive
        :return: None
        """
        pass

    def rotate(self, angle):
        """
        Rotate in place
        :param angle: angle to rotate
        :return: None
        """
        pass
    
    def convert_to_euler(self, quat):
        # type: (Quaternion) -> float
        """
        This might be helpful to have
        :param quat: quaternion 
        :return: euler angles
        """
        

def main():
    """_summary_
    create all the node start up here
    """
    
    




if __name__ == '__main__':
    main()