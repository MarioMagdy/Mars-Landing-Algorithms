import math
import csv

def acc_data(filename, dt):

  vx = 0
  vy = 0
  vz = 0
  px = 0
  py = 0
  pz = 0

  with open(filename, 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    next(csvreader) 
    for row in csvreader:

      ax = float(row[0])
      ay = float(row[1])
      az = float(row[2])


      vx += ax * dt
      vy += ay * dt
      vz += az * dt


      px += vx * dt
      py += vy * dt
      pz += vz * dt


  data = [vx, vy, vz, px, py, pz]
  return data
















