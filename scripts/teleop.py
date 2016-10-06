#!/usr/bin/python

import sys,pygame,math,time
import rospy
from std_msgs.msg import Float64

class Mode:
    Stop,Left,Right,Drive = range(4)


def main():
    mode = Mode.Stop
    drivespeed = 0    

    pygame.init()
    size=width,height=320,320
    black=0,0,0
    screen = pygame.display.set_mode(size)
    font = pygame.font.Font(None,18)

    publeft = rospy.Publisher('leftmotor',Float64,queue_size=10)
    pubright = rospy.Publisher('rightmotor',Float64,queue_size=10)
    rospy.init_node('teleop',anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    mode = Mode.Left
                elif event.key == pygame.K_RIGHT:
                    mode = Mode.Right
                elif event.key == pygame.K_UP:
                    if mode == Mode.Drive:
                        drivespeed = drivespeed+1
                        if drivespeed==0:
                            mode = Mode.Stop
                    else:
                        drivespeed = 1
                        mode = Mode.Drive
                elif event.key == pygame.K_DOWN:
                    if mode == Mode.Drive:
                        drivespeed = drivespeed-1
                        if drivespeed==0:
                            mode = Mode.Stop
                    else:
                        drivespeed = -1
                        mode = Mode.Drive
                elif event.key == pygame.K_SPACE:
                    mode = Mode.Stop
                elif event.key == pygame.K_q:
                    pygame.quit()
                    sys.exit()
            elif event.type == pygame.KEYUP:
                if mode==Mode.Left or mode == Mode.Right:
                    mode = Mode.Stop
                
        if mode == Mode.Stop:
            leftmotor=0
            rightmotor=0
        elif mode == Mode.Left:
            leftmotor = -1
            rightmotor = 1
        elif mode == Mode.Right:
            leftmotor = 1
            rightmotor = -1
        elif mode == Mode.Drive:
            leftmotor = drivespeed
            rightmotor = drivespeed
        leftmotor = leftmotor * 0.1
        rightmotor = rightmotor * 0.1
        print "%d: %f,%f" % (mode,leftmotor,rightmotor)
        publeft.publish(Float64(leftmotor))
        pubright.publish(Float64(rightmotor))
        rate.sleep()
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
