# ``mdr_perceive_plane_action``

An action for turning the camera to a specific location.

## Action definition

### Goal:


### Result:


### Feedback:


## Directory structure

```
├── CMakeLists.txt
├── package.xml
├── README.md
├── ros
│   ├── action
│   │   └── LookAt.action
│   ├── config
│   ├── launch
│   │   ├── look_at_client.launch
│   │   ├── look_at.launch
│   │   └── look_at_test.launch
│   ├── scripts
│   │   ├── look_at_action
│   │   ├── look_at_client
│   │   └── look_at_client_test
│   └── src
│       └── mdr_look_at_action
│           ├── action_states.py
│           └── __init__.py
└── setup.py
```

## Launch file parameters

### Action server

The following arguments may be passed when launching the action server:


### Action client


The following parameters need to be passed when launching the action client:

## Dependencies

* ``mdr_object_recognition_mean_circle``
* ``mas_perception_libs``
* ``mdr_object_recognition``
* ``mcr_dynamic_reconfigure_client``
* ``mcr_perception_msgs``

## Example usage
