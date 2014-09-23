int32 val
#goal
int32 GOAL_PICKUP_FRONT=1
int32 GOAL_DEPLOY_FRONT=2
int32 GOAL_PICKUP_REAR=11
int32 GOAL_DEPLOY_REAR=12
#result
int32 RESULT_SUCCESS=3 #everything went fine
int32 RESULT_DETECTION_FAILED=4 #could not detect the sensornode
int32 RESULT_SOFT_MANIPULATION_FAILURE=5 #i'm in home position
int32 RESULT_HARD_MANIPULATION_FAILURE=6 #i need help from a human
int32 RESULT_LOGICAL_FAILURE=7 #Deploy when no sensornode is loaded #Pickup when 2 sensornodes are loaded
int32 RESULT_STARTSTATE_FAILURE=8
