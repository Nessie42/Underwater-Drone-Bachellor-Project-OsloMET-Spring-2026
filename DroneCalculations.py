
import numpy as np
import pandas as pd

#CONSTANTS:
g = 9.81 #m/s^2
air_density = 1.225 #kg/m^3
water_density = 1025 #kg/m^3
drag_coefficient_water = 0.0015
time_step = 1/6 #10s

#HULL:
skrog_diameter = 0.105 #m
skrog_length = 0.3 + 0.1 #m
skrog_mass = 0.35*0.51 + 0.17 + 0.06 #kg 

ballast_diameter = 0.105 #m
ballast_length = 0.30 + 0.1 #m
ballast_mass = 0.15*0.51 + 0.17 + 0.06 #kg

frame_diameter = 0.032 #m
frame_length = 16*0.250 + 20*0.5 #m
#frame_mass = 16*0.054 + 20*0.138 + 24*0.028 + 8*0.038
frame_mass = 0

floaters_mass = 0.06 #kg/2m
floaters_volume = 0.0077 #m^3/2m
floaters_length = 0.5 #m


internals_mass = 0.745 + 0.5 #kg

extra_mass = 5 #kg

#characteristics:
set_point_depth = 10 #m
full_ballast_tank_percentage = 0.5
inner_tube_diameter = 0.0048 #m
number_of_tubes = 1
inner_radius_pump = 0.06 #m

initial_rpm = 4700


#gear_ratios = [(10,115),(25,50),(15,75)]

gear_ratios = [(1,50)]


#FUNCTIONS:
def BuoyantForce(density, volume):
    buoyant_force = density*volume*g
    return buoyant_force

def Weight(mass):
    weight = mass*g
    return weight

def Density(mass, volume):
    density = mass/volume
    return density

def BallastPressure(ballast_volume, water_volume):
    initial_pressure = 100000 #Pa
    current_air_volume = ballast_volume-water_volume #m^3
    volume_ratio = ballast_volume/current_air_volume
    current_pressure = initial_pressure*volume_ratio #Pa
    return current_pressure

def FillTime(flow_rate, ballast_volume):
    ballast_fill_time = (ballast_volume/flow_rate)*full_ballast_tank_percentage #min
    return ballast_fill_time

def FinalRPM(*gears):
    ratios = []
    for ratio in (gears):
        for i,t in (ratio):
            ratio = i/t
            ratios.append(ratio)
    final_ratio = 1
    for i in ratios:
        final_ratio = i*final_ratio
    rpm = initial_rpm*final_ratio
    
    return rpm
    
def FlowRate(pump_rpm):
    tube_volume = np.pi*(inner_tube_diameter/2)**2*inner_radius_pump
    volume_flow_rate = tube_volume*pump_rpm*number_of_tubes
    return volume_flow_rate
    
def SinkTime(mass):
    floaters_buoyancy = 0.5*BuoyantForce(floaters_density, floaters_volume)
    buoyant_force = BuoyantForce(water_density, total_volume)
    weight = Weight(mass)
    
    if buoyant_force+floaters_buoyancy >= weight:
        return "N/A"
    elif buoyant_force+floaters_buoyancy == weight:
        return "N/A"
    else:
        net_force = weight - buoyant_force - floaters_buoyancy
    
        actual_acceleration = net_force/mass
    
        minutes = np.sqrt((2*set_point_depth)/actual_acceleration)

        return minutes

def VolCylinder(diameter, height):
    volume = np.pi*((diameter/2)**2)*height
    return volume

def SinkWeight():
    floaters_buoyancy = BuoyantForce(floaters_density, floaters_volume)
    buoyant_force = BuoyantForce(water_density, total_volume)
    mass= (buoyant_force + floaters_buoyancy)/g
    
    return mass
    
    

#CALCULATIONS:
volume_skrog = VolCylinder(skrog_diameter,skrog_length)
volume_ballast = VolCylinder(ballast_diameter,ballast_length)
#volume_frame = VolCylinder(frame_diameter,frame_length)
volume_frame = 0

total_volume = volume_skrog + volume_ballast + volume_frame #m^3

total_mass = skrog_mass + ballast_mass + frame_mass + internals_mass + extra_mass #kg

floaters_density = Density(floaters_mass, floaters_volume)



final_rpm = round(FinalRPM(gear_ratios))
gear_ratio = "1:"+str(round(initial_rpm/final_rpm))
flow_rate = round(FlowRate(final_rpm),5)

empty_ballast_drone_density = round(Density(total_mass, total_volume),2)

full_ballast_mass = round((volume_ballast*full_ballast_tank_percentage*water_density)+total_mass,2)
full_ballast_drone_density = round(Density(full_ballast_mass, total_volume),2)
full_ballast_pressure = round(BallastPressure(volume_ballast, (full_ballast_tank_percentage*volume_ballast)))

fill_time = round(FillTime(flow_rate, volume_ballast),1)
sink_time = SinkTime(full_ballast_mass)

if isinstance(sink_time,str):
    sink_yn = "No"
else:
    sink_yn = "Yes"

#RESULTS:

data = {"Does it sink?" : sink_yn,
        "Sinks if heavier than:" : [SinkWeight()],
           "Hull Mass (kg):":skrog_mass + frame_mass + internals_mass, 
           "Extra Mass (kg):":extra_mass, 
           "Total Mass (kg):":total_mass,
           "Hull Volume (L):": [(volume_skrog + volume_frame)*1000],
           "Ballast Volume (L):": [(volume_ballast)*1000],
           "Total Volume (L):": [total_volume*1000],
           "Gear Ratio:": gear_ratio,
           "Nr. of Pumps:": number_of_tubes,
           "Flow Rate (L/min):": [flow_rate*1000],
           "Ballast Fill Time (min):": fill_time,
           "Sink Time (min):": sink_time}

df = pd.DataFrame(data)
transposed_df = df.T

print(transposed_df)


print(f"With {number_of_tubes} pump tube(s), and a gear ratio of {gear_ratio}, the pump rpm is {final_rpm} and the flow rate of the pump is {flow_rate} m^3/m, or {round(flow_rate*1000,5)}L/m")

print(f"With a flow rate of {flow_rate} m^3/m, a ballast volume of {volume_ballast} m^3 which is full at {full_ballast_tank_percentage*100}% capacity; It will take {fill_time} minutes for the tank to fill.")


if full_ballast_drone_density > water_density:
    print(f"The drone density at full ballast tank is {full_ballast_drone_density}, and the water density is {water_density}, and so the drone will sink.")
elif full_ballast_drone_density == water_density:
    print(f"The drone density at full ballast tank is {full_ballast_drone_density}, and the water density is {water_density}, and so the drone is neutrally bouyant.")
else:
    print(f"The drone density at full ballast tank is {full_ballast_drone_density}, and the water density is {water_density}, and so the drone will float.")
    
print(f"With this density and weight, it will take the drone {sink_time} minutes to reach {set_point_depth} meters depth.")
    
print(f"With a full ballast tank at {full_ballast_tank_percentage*100}%, the internal air pressure is: {full_ballast_pressure}. Which means the pump experiences a pressure of {round(full_ballast_pressure-200000)}")

