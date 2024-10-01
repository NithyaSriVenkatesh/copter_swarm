import csv
import simplekml
from geopy.distance import distance
from geopy.point import Point


def CreateGridsForSpecifiedAreaAndSpecifiedDrones(center_latitude,center_longitude,num_of_drones,grid_space,coverage_area,dir_path):
    print("!!!!!!!!!!!")
    center_lat = center_latitude 
    center_lon = center_longitude  


    num_rectangles = num_of_drones 
    grid_spacing = grid_space 
    #meters_for_extended_lines = 200

    full_width,full_height =  coverage_area,coverage_area  

    rectangle_height = full_height / num_rectangles

    center_point = Point(center_lat, center_lon)

    west_edge = distance(meters=full_width / 2).destination(center_point, 270)
    east_edge = distance(meters=full_width / 2).destination(center_point, 90)

    for i in range(num_rectangles):
        top_offset = (i * rectangle_height) - (full_height / 2) + (rectangle_height / 2)
        
        top_center = distance(meters=top_offset).destination(center_point, 0)
        top = distance(meters=rectangle_height / 2).destination(top_center, 0)
        bottom = distance(meters=rectangle_height / 2).destination(top_center, 180)

        kml = simplekml.Kml()

        csv_data = []

        current_lat = bottom.latitude
        line_number = 0 
        line = kml.newlinestring()
        line.altitudemode = simplekml.AltitudeMode.clamptoground
        line.style.linestyle.color = simplekml.Color.red
        line.style.linestyle.width = 2
        waypoint_number = 1

        while current_lat <= top.latitude:
            line_number += 1  
            current_point = Point(current_lat, west_edge.longitude)
            east_point = distance(meters=full_width).destination(current_point, 90)
            if line_number % 2 == 1:
                csv_data.append((current_point.latitude, current_point.longitude))
                csv_data.append((east_point.latitude, east_point.longitude))

                line.coords.addcoordinates([(current_point.longitude, current_point.latitude),
                            (east_point.longitude, east_point.latitude)])
                kml.newpoint(name=f"{waypoint_number}", coords=[(current_point.longitude, current_point.latitude)])
                waypoint_number += 1
                kml.newpoint(name=f"{waypoint_number}", coords=[(east_point.longitude, east_point.latitude)])
                waypoint_number += 1
            else:
                csv_data.append((east_point.latitude, east_point.longitude))
                csv_data.append((current_point.latitude, current_point.longitude))

                line.coords.addcoordinates([(east_point.longitude, east_point.latitude),(current_point.longitude, current_point.latitude)])
                kml.newpoint(name=f"{waypoint_number}", coords=[(east_point.longitude, east_point.latitude)])
                waypoint_number += 1
                kml.newpoint(name=f"{waypoint_number}", coords=[(current_point.longitude, current_point.latitude)])
                waypoint_number += 1
            
            current_lat = distance(meters=grid_spacing).destination(current_point, 0).latitude

        kml_filename = f"search-drone-{i+1}.kml"
        kml.save(dir_path+"/"+kml_filename)

        csv_filename = f"d{i+1}.csv"

        with open(dir_path+"/"+csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Latitude", "Longitude"])
            writer.writerows(csv_data)
    print("BBBBB")
    return 1

center_lat =  13.389460 
center_lon =  80.233607 

num_of_drones = 3 
grid_spacing = 8  
coverage_area = 100 

#CreateGridsForSpecifiedAreaAndSpecifiedDrones(center_lat,center_lon,num_of_drones,grid_spacing,coverage_area)
