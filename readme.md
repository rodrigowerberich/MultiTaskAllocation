# Multi robot - multi task allocation with communication restriction

A group of robots (**R**)  can be described by two features, 
* its type (**robot_type**) and 
* its position (**robot_position**).

In order to identify them, we also give them a name (**robot_name**). The types each robot can have are all predetermined (**robot_types**).

This group of robots wishes to accomplish a group of missions (**missions**). These missions can be described by
* their set of tasks (**mission_tasks**),
* their priority (**mission_priority**) and
* their deadline (**mission_deadline**)

Tasks are steps that need to be executed in order to fulfill a mission, they can be defined by
* a set of prerequisite tasks (**task_prerequisites**),
* a position (**task_position**) and
* a type (**task_type**).

The types each task can have is also predetermined (**task_types**).

Robots and tasks types are used to determine the amount of effort a robot must execute in order to accomplish a task (**effort_function**).

The task type is also used in order to compute the reward a certain task gives.

To plan something, we must also define the world in which we whish to plan. Our world has only 3 information that we need in order to plan, 
* the area we want to search (**search_area**), 
* the obstructed area (**obstructed_area**) and
* the connectivity probability of the are (**connectivity_function**).

This is what we will use to define our problem. 

We will develop a program that will received a json file as input as will output another json file with the solution.

## Input JSON
The input json must contain these keys, they are all we need in order to define our problem. 
```json
{
    "robot_types":[],
    "task_types":[],
    "effort_function":[],
    "reward_function":[],
    "robots":[],
    "tasks":[],
    "missions":[],
    "search_area":{},
    "obstructed_area":{},
    "connectivity_function:":{}    
}
```

Each one of these keys must have a specific value, and some accept multiple values.

### Robot types

The robot types key must always be an array of strings, each string represent a different type, there must be no repeat in the type, repetition will be ignored!

```json
{
    "robot_types": ["type 1", "type 2", ..., "type n"]
}
```

### Task types

Follows the same convention as the robot types

```json
{
    "task_types": ["type 1", "type 2", ..., "type n"]
}
```

### Effort function

The effort function must be an array of json objects. It must have ***#task_types X #robot_types*** elements, where #task_types and #robot_types are the number of task types and the number of robot types, respectively.

#### Effort function objects
All effort function objects have the following pattern:
```json
{
    "task_type":"type",
    "robot_type":"type",
    "function": FunctionJSONObject
}
```
The function key must contain a function json object. (See auxiliary objects).

### Reward function

The reward function must be an array of json objects. It must have ***#task_types*** elements.

#### Reward function objects
All reward function objects have the following pattern:
```json
{
    "task_type":"type",
    "function": FunctionJSONObject
}
```
The function key must contain a function json object. (See auxiliary objects).

### Robots

The robots key must be an array of json objects. There is no restriction on the number of robots that can exist.

#### Robots objects
All robots objects have the following pattern:
```json
{
    "name": "Robot Name",
    "type": "Type",
    "position" : PositionJSONObject
}
```

The position key must contain a position json object. (See auxiliary objects).

### Tasks

The tasks key must be and array of json objects. There is no restriction on the number of tasks that can exist.

#### Tasks objects
All tasks objects have the following pattern:
```json
{
    "id": "id",
    "type": "type",
    "position": PositionJSONObject,
    "prerequisites": ["id1", "id2", ..., "idN"]
}
```

There can not be any ciclic reference in the prerequisites.

### Missions

The missions key must be an array of json objects. There can be at most ***#tasks*** missions. A task can not be in more than one mission at a time. A task can not be in a different mission from its prerequisites.

#### Mission objects
All mission objects have the following pattern:
```json
{
    "id": "id",
    "priority": number,
    "deadline": number,
    "mission_tasks": ["task_id_1", "task_id_2", ..., "task_id_n"]
}
```

### Search area

The search area object must restrict the search area for the planning algorithm.
These are the valid objects:
1. Any of the geometric objects
    
    The search area will be delimited to the interior of the geometric object
    See Auxiliary objects

### Obstructed area

The obstructed area are barriers and obstacles that restrict the robots movements.
These are the valid objects:
1. Collection of geometric objects

    ```json
    {
        "type":"geometric",
        "obstacles": [ GeometricJSONObject, GeometricJSONObject, ..., GeometricJSONObject]
    }
    ```
2. A png image
    ```json
    {
        "type":"png",
        "path": "absolute_path",
        "resolution": number // units per pixel
    }
    ```
### Connectivity function

These are the valid objects:
1. Collection of geometric objects

    ```json
    {
        "type":"geometric",
        "connectivity_areas": [ GeometricJSONObject, GeometricJSONObject, ..., GeometricJSONObject]
    }
    ```
2. A png image
    ```json
    {
        "type":"png",
        "path": "absolute_path",
        "resolution": number // units per pixel
    }
    ```



### Auxiliary objects

#### 1) Function objects
For now the available function objects are:
1. Simple value:
    ```json
    { 
        "type": "simple_value",
        "value": 0.9
    }
    ```
2. Infinity:
    ```json
    { 
        "type": "infinity",
    }
    ```

#### 2) Position objects
For now the available position objects are:
1. 2d position:
    ```json
    {
        "type":"2d",
        "x": 0.0,
        "y": 0.0
    }
    ```
#### 3) Geometric objects
1. Rectangle
    Describes a rectangle of dimension width X height. The rectangle position can be defined by any of its corners or its center. If no positional argument is given, bottom left corner is assumed with 0,0 value.
    ```json
    {
        "type":"rectangle",
        "bottom_left": 2dPositionObject || "top_left": 2dPositionObject || "bottom_right": 2dPositionObject || "top_right": 2dPositionObject || "center": 2dPositionObject , 
        "width": number,
        "height": number,
    }
    ```
2. Circle
    Describes a circle with given radius and center position. If no center position is given, 0,0 is assumed.
    ```json
    {
        "type":"circle",
        "center": 2dPositionObject,
        "radius": number
    }
    ```

### Example Input Json
```json
{
    "robot_types":["r1", "r2"],
    "task_types":["t1", "t2"],
    "effort_function":[
        {
            "task_type":"t1",
            "robot_type":"r1",
            "function": { 
                "type": "simple_value",
                "value": 5
            }
        },
        {
            "task_type":"t1",
            "robot_type":"r2",
            "function": { 
                "type": "simple_value",
                "value": 4
            }
        },
        {
            "task_type":"t2",
            "robot_type":"r1",
            "function": { 
                "type": "simple_value",
                "value": 1
            }
        },
        {
            "task_type":"t2",
            "robot_type":"r2",
            "function": { 
                "type": "infinity",
            }
        },
    ],
    "reward_function":[
        {
            "task_type":"t1",
            "function": {
                "type": "simple_value",
                "value": 20
            }
        },
        {
            "task_type":"t2",
            "function": {
                "type": "simple_value",
                "value": 20
            }
        }
    ],
    "robots":[
        {
            "name": "Bruce",
            "type": "r1",
            "position" : {
                "type": "2d",
                "x": 5.0,
                "y": 3.7
            }
        },
        {
            "name": "Robert",
            "type": "r2",
            "position" : {
                "type": "2d",
                "x": 3.0,
                "y": -1.6
            }
        }
    ],
    "tasks":[ 
        {
            "id": "t1",
            "type": "t1",
            "position": {
                "type": "2d",
                "x": -2,
                "y": -6
            },
            "prerequisites": []
        },
        {
            "id": "t2",
            "type": "t1",
            "position": {
                "type": "2d",
                "x": 7,
                "y": -3
            },
            "prerequisites": []
        },
        {
            "id": "t3",
            "type": "t2",
            "position": {
                "type": "2d",
                "x": -1,
                "y": 3
            },
            "prerequisites": ["t1", "t2"]
        },
        {
            "id": "t4",
            "type": "t1",
            "position": {
                "type": "2d",
                "x": -2,
                "y": 6
            },
            "prerequisites": []
        },
        {
            "id": "t5",
            "type": "t1",
            "position": {
                "type": "2d",
                "x": 6,
                "y": -6
            },
            "prerequisites": ["t4"]
        },
        {
            "id": "t6",
            "type": "t1",
            "position": {
                "type": "2d",
                "x": -6,
                "y": 6
            },
            "prerequisites": ["t4"]
        },
        {
            "id": "t7",
            "type": "t2",
            "position": {
                "type": "2d",
                "x": 0,
                "y": 0
            },
            "prerequisites": ["t5", "t6"]
        },
        {
            "id": "t8",
            "type": "t2",
            "position": {
                "type": "2d",
                "x": -2,
                "y": 0
            },
            "prerequisites": []
        }
     ],
    "missions":[ 
        {
            "id": "m1",
            "priority": 1,
            "deadline": 100,
            "mission_tasks": ["t1", "t2", "t3", "t8"]
        },
        {
            "id": "m2",
            "priority": 1,
            "deadline": 150,
            "mission_tasks": ["t4", "t5", "t6", "t7"]
        }
     ],
    "search_area":{
        "type":"rectangle",
        "center": {
            "type": "2d",
            "x": 0.0,
            "y": 0.0
        },
        "width": 20,
        "height": 20,
    },
    "obstructed_area":{
        "type":"geometric",
        "obstacles": [ 
            {
                "type":"rectangle",
                "bottom_left": {
                    "type": "2d",
                    "x": -2,
                    "y": -2
                },
                "width": 1,
                "height": 8
            }
        ]
    },
    "connectivity_function:":{
        "type":"geometric",
        "connectivity_areas": [ 
            {
                "type":"circle",
                "radius": 5
            }
        ]
    }  
}
```

## Output JSON
// TODO
```json
[
    { 
        "robot_name":"", 
        "plan":[ 
            {
                "task_name":"", 
                "start": ,
                "stop": 
            }, 
            // ...
            {
                "task_name":"", 
                "start": ,
                "stop": 
            }
        ]
    },
    // ...
    { 
        "robot_name":"", 
        "plan":[ 
            {
                "task_name":"", 
                "start": ,
                "stop": 
            }, 
            // ...
            {
                "task_name":"", 
                "start": ,
                "stop": 
            }
        ]
    },
]
```

## Task definition:
```json
{
    "robot_types":["a1", "a2", "a3", "a4", "a5"],
    "task_types":["t1", "t2", "t3", "t4"],
    "effort_function":{
        "a1":{
            "t1": 15,
            "t2": 50,
            "t3": 10,
            "t4": "inf"
        },
        "a2":{
            "t1": 50,
            "t2": 20,
            "t3": 15,
            "t4": 1
        },
        "a3":{
            "t1": 9,
            "t2": 7,
            "t3": 70,
            "t4": 35
        },
        "a4":{
            "t1": 25,
            "t2": "inf",
            "t3": 1,
            "t4": 3
        },
        "a5":{
            "t1": 25,
            "t2": "inf",
            "t3": 1,
            "t4": 3
        }
    },
    "reward_function":{
        "t1": 15,
        "t2": 54,
        "t3": 17,
        "t4": 25
    }
}
```

```json
{
    "micro tasks": {
        "1": {
            "pos": [
                -6.895000000000001,
                0.13333333333333144
            ],
            "previous": [],
            "type": "t1"
        },
        "2": {
            "pos": [
                -4.000000000000001,
                -3.0
            ],
            "previous": [
                "1",
                "3"
            ],
            "type": "t1"
        },
        "3": {
            "pos": [
                4.000000000000001,
                3.0
            ],
            "previous": [
                "4"
            ],
            "type": "t2"
        },
        "4": {
            "pos": [
                -7.0,
                8.0
            ],
            "previous": [],
            "type": "t3"
        }
    },
    "robots": {
        "1": {
            "pos": [
                5.901249999999997,
                7.866666666666666
            ],
            "type": "a1"
        },
        "2": {
            "pos": [
                -3.0,
                5.0
            ],
            "type": "a3"
        },
        "3": {
            "pos": [
                3.0,
                -5.0
            ],
            "type": "a4"
        }
    },
    "tasks": {
        "1": {
            "deadline": 0.0,
            "micro tasks": [
                "1",
                "3",
                "2",
                "4"
            ],
            "priority": 1.0
        }
    }
}
```