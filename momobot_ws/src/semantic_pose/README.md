# semantic_pose

This package allows a user to label specific areas in the map so robots can represent its location in a more human-readable way.

### When is this package useful?
- Trigger Point - when you want the robot to do something every time it reaches a certain area.

- Intuitive Representation - sometimes, it's just more useful to know that the robot is in the kitchen or living room rather than knowing its coordinates.

Boost's [within](https://www.boost.org/doc/libs/1_62_0/libs/geometry/doc/html/geometry/reference/algorithms/within/within_2.html) library is used to constantly check if the robot is contained in an area(polygon).

## Defining Areas in the Map

Each area in the map can be labeled by defining the vertices of the boundary(polygon sides) in config/config.yaml file.


    places:
    - name: "place1"
      boundary: [[-5.01995038986, 0.341648638248], [ -5.02670955658,-2.89252948761], [-6.98518037796, -2.95654678345], [-7.06083536148, 0.271802127361]]

    - name: "place2"
      boundary: [[-7.01539182663, 0.339973062277],[-1.44279897213 ,0.283673077822],[ 4.77711629868,6.89892435074],[4.77711677551 ,9.23537540436],[-5.74889469147 ,9.23537540436],[ -8.45075798035,6.56112480164]]

    - name: "place3"
      boundary: [[4.73916625977,6.79155063629],[4.66398239136,3.70972466469],[2.89470362663,1.91181647778]]



Each label consists of two parameters:

'name' - Arbitrary name of an area (ie. kitchen).

'boundary' - Array of vertices of a polygon to represent the boundary of an area.

You can grab the vertices by using 'Publish Point' in Rviz and subscribing to '/clicked_point'. You can do this in a clockwise or an anti-clockwise manner.

Take note that the number of places and vertices in a boundary is also arbitrary.

## Package Configuration

The robot constantly checks it's transform between the map frame and robot's frame. Fill in the base_frame_id and map_frame_id accordingly:

    <node pkg="semantic_pose" type="semantic_pose_node" name="semantic_pose" output="screen">
        <param name="base_frame_id" value="base_footprint" />
        <param name="map_frame_id" value="map" />
        <param name="publish_rate" value="10.0" />
    </node>

You can also change how often the robot's location is published with 'publish_rate' parameter.

## Running the package

Launch the package:

    roslaunch semantic_pose semantic_pose.launch

Subscribe to 'location' topic to know which area the robot is in:

    rostopic echo location













