import math
import csv

def gyro_data(filename):

  roll = 0
  pitch = 0
  yaw = 0


  with open(filename, 'r') as csvfile:
    csvreader = csv.reader(csvfile)
   
    for row in csvreader:
      dt = float(row[0])
      gx = float(row[1])
      gy = float(row[2])
      gz = float(row[3])


      roll += gx * dt
      pitch += gy * dt
      yaw += gz * dt

 
  attitude = [roll, pitch, yaw]

  return attitude
