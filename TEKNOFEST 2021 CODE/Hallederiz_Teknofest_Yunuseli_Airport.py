import time
from dronekit import connect, VehicleMode, Command,LocationGlobalRelative,LocationGlobal
from pymavlink import mavutil
import argparse
import cv2
import numpy as np
from threading import Thread
import math
#import RPi.GPIO as GPIO



parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='tcp:127.0.0.1:5762')
args = parser.parse_args()

connection_string = args.connect
vehicle=connect(connection_string,wait_ready=True)


"""
vehicle = connect("/dev/ttyACM0", wait_ready=True)
vehicle.mode = VehicleMode("GUIDED")
"""

frameWidth = 640
frameHeight = 480

deadZone = 75

cxArray = list()
cyArray = list()

check = False
counter = 0

"""

servoPIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)
p = GPIO.PWM(servoPIN, 50) # GPIO 17 for PWM with 50Hz
p.start(2.5) # Initialization

"""


direk1_sag = LocationGlobalRelative(40.2304231, 29.0095780,10)
direk1_ust = LocationGlobalRelative(40.2305265, 29.0093223,10)
kirmizi_start = LocationGlobalRelative(40.2304149, 29.0091203,10)
kirmizi_finish = LocationGlobalRelative(40.2297883, lp,10)
direk2_sol = LocationGlobalRelative(40.2296768, 29.009197,10)
direk2_alt = LocationGlobalRelative(40.229629, 29.0093373,10)
direk2_sag = LocationGlobalRelative(40.2297933, 29.0095608,10)
havuza_doru = LocationGlobalRelative(40.2302148, 29.0095544,10)
havuz_ileri = LocationGlobalRelative(40.2305003,29.0095462,10)
havuz = LocationGlobalRelative(40.2304075,29.0095394,10)
havuz5 = LocationGlobalRelative(havuz.lat,havuz.lon,5)
havuz3 = LocationGlobalRelative(havuz.lat,havuz.lon,3)
havuz2 = LocationGlobalRelative(havuz.lat,havuz.lon,2)
havuz1_5 = LocationGlobalRelative(havuz.lat,havuz.lon,1.5)
havuz1_3 = LocationGlobalRelative(havuz.lat,havuz.lon,1.3)
target = LocationGlobalRelative(0,0,0)
land = LocationGlobalRelative(40.2301957, 29.0097947,10)

"""

def servo_ac():
    p.ChangeDutyCycle(7.5)
def servo_kapa():
    p.ChangeDutyCycle(4)


"""

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print ("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)

    print ("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)

    time.sleep(2)
    print ("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("Reached target altitude")
            break
        time.sleep(1)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def gidici(point2,string,distance_kalan = 5.0):
    vehicle.simple_goto(point2)
    print(string+"'e gidiliyor")
    while True:
        inst_loc = vehicle.location.global_relative_frame
        distance = get_distance_metres(inst_loc,point2)
        if distance <= distance_kalan:
            break
        else:
            time.sleep(0.5)


def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(0.025)


def vectorcu():
    if len(cxArray) != 0 and len(cyArray) != 0:
        x_vector = 0
        y_vector = 0
        toplamx = 0
        toplamy = 0
        factor = 250
        for x in cxArray:
            toplamx += x
        for y in cyArray:
            toplamy += y
        x_vector = (toplamx / len(cxArray)) / factor
        y_vector = (toplamy / len(cyArray)) / factor
        send_ned_velocity(x_vector, y_vector, 0, 1)
        cxArray.clear()
        cyArray.clear()


def display(img):
    pt1 = (275,195)
    pt2 = (365,285)
    cv2.line(img,(int(frameWidth/2),0),(int(frameWidth/2),frameHeight),(51,255,51),2)
    cv2.line(img, (0, int(frameHeight / 2) ), (frameWidth, int(frameHeight / 2) ), (51,255,51), 2)
    cv2.rectangle(img,pt1,pt2,(51,255,51),2)


def opencv():
    global target
    global check
    global counter

    webcam = cv2.VideoCapture(0)
    while not check:

        _, imageFrame = webcam.read()

        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

        # -------------------------raspberry
        red_lower = np.array([136, 87, 111], np.uint8)
        red_upper = np.array([180, 255, 255], np.uint8)
        # ----------------------------pc
        # red_lower = np.array([0, 139, 139], np.uint8)
        # red_upper = np.array([179, 255, 255], np.uint8)
        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

        contours, hierarchy = cv2.findContours(red_mask,
                                               cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            areaMin = 3000
            if (area > areaMin):
                x, y, w, h = cv2.boundingRect(contour)
                imageFrame = cv2.rectangle(imageFrame, (x, y),
                                           (x + w, y + h),
                                           (0, 0, 255), 2)

                cx = int(x + (w / 2))
                cy = int(y + (h / 2))
                vx = cx - (frameWidth / 2)
                vy = (frameHeight / 2) - cy
                cv2.circle(imageFrame, (cx, cy), 4, (51, 255, 51), 1)

                if 45 >= vx >= -45 and vy <= 45 and vy >= -45:
                    counter += 1
                    if counter >= 100:
                        print("Deadzone")
                        target = vehicle.location.global_relative_frame
                        print("Alan Kaydedildi")
                        print(target)
                        vehicle.airspeed = 5
                        gidici(direk2_sol, "Direk2_sol", 1)
                        check = True

                else:
                    if vy >= 0 and vx <= 0 or vy <= 0 and vx >= 0:
                        cxArray.append(vx)
                        cyArray.append(vy)

                    else:
                        cxArray.append(-vx)
                        cyArray.append(-vy)

                vectorcu()

        display(imageFrame)
        cv2.imshow("Color Tracking", imageFrame)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            webcam.release()
            cv2.destroyAllWindows()
            break


th2 = Thread(target=opencv)


def ilktur():
    try:
        global target
        arm_and_takeoff(10)
        vehicle.airspeed = 5
        gidici(direk1_sag,"Direk 1'in sagi")
        gidici(direk1_ust,"Direk1_ust")
        gidici(kirmizi_start,"Kırmızı Start")
        vehicle.airspeed = 2
        vehicle.simple_goto(kirmizi_finish)
        th2.start()
        th2.join()
        gidici(direk2_sol,"Direk 2'sol")
        gidici(direk2_alt,"Direk 2 Alt")
        gidici(direk2_sag,"Direk 2 Sag")
        gidici(havuza_doru,"Havuza Doğru")
        gidici(havuz_ileri,"Havuz ileri",1)
        vehicle.airspeed = 1.5
        gidici(havuz,"Havuza Gidiliyor")
        print("Havuza Varıldı.")
        time.sleep(2)
        gidici(havuz5,"5 Metreye Alçalınıyor.",0.3)
        print("5 Metre")
        time.sleep(3)
        gidici(havuz3, "3 Metreye Alçalınıyor.", 0.3)
        print("3 Metre")
        time.sleep(3)
        gidici(havuz2, "2 Metreye Alçalınıyor.", 0.3)
        print("2 Metre")
        time.sleep(3)
        gidici(havuz1_5, "1.5 Metreye Alçalınıyor.", 0.3)
        print("1.5 Metre")
        time.sleep(5)
        gidici(havuz1_3, "1.3 Metreye Alçalınıyor.", 0.3)
        print("1.3 Metre")
        time.sleep(5)
        #--------------SU ALMA----------------
        print("SERVO AÇILDI.")
        #servo_ac()
        time.sleep(20)
        print("SERVO KAPATILDI")
        #servo_kapa()
        time.sleep(5)
        #--------------------------------------
        gidici(havuz, "Havuz Yükseliş", 0.3)
        time.sleep(3)
        print("Yükselme Tamamlandı")
        print("Alana Gidiliyor.")
        vehicle.airspeed = 2
        gidici(target, "Target", 0.3)
        print("Alana Varıldı.")
        time.sleep(2)
        target2 = LocationGlobalRelative(target.lat, target.lon, 3)
        gidici(target2, "Alçalınıyor", 0.3)
        time.sleep(2)
        # ---------------SU BIRAKMA------------
        print("SERVO AÇILDI.")
        #servo_ac()
        time.sleep(20)
        print("SERVO KAPATILDI")
        #servo_kapa()
        time.sleep(5)
        # --------------------------------------------
        print("Yük Boşaltıldı.Devam Ediliyor.")
        gidici(target,"Target Yükseliş",0.3)
        time.sleep(2)
        vehicle.airspeed = 5
        gidici(direk2_sol, "Direk 2'nin solu")
        gidici(direk2_alt, "Direk 2 Alt")
        gidici(direk2_sag, "Direk 2 Sag")
        gidici(land,"Land Noktasına Gidiliyor")
        print("Land Noktasına Varıldı.Görev Tamamlandı.")
        time.sleep(60)

    except:
        print("Şekerim düştü.Bayılmaca.")
        vehicle.mode = VehicleMode("LAND")

    finally:
        time.sleep(2)
        vehicle.armed = False
        time.sleep(2)
        vehicle.close()


th1 = Thread(target=ilktur)

th1.start()
th1.join()
