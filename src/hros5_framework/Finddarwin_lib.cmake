SET(DARWIN_COMM_HEADERS Action.h BallFollower.h BallTracker.h Camera.h CM730.h ColorFinder.h DARwIn.h FSR.h Head.h Image.h ImgProcess.h JointData.h Kinematics.h Matrix.h minIni.h MotionManager.h MotionModule.h Servo.h Plane.h Point.h Vector.h Walking.h)

FIND_PATH(darwin_comm_INCLUDE_DIR ${DARWIN_COMM_HEADERS} ${DARWIN_LIB_PATH}/Linux/include ${DARWIN_LIB_PATH}/Framework/include)

SET(DARWIN_LINUX_HEADERS LinuxActionScript.h LinuxCamera.h LinuxCM730.h LinuxDARwIn.h LinuxMotionTimer.h LinuxNetwork.h mjpg_streamer.h)

FIND_PATH(darwin_linux_INCLUDE_DIR ${DARWIN_LINUX_HEADERS} ${DARWIN_LIB_PATH}/Linux/include ${DARWIN_LIB_PATH}/Framework/include)

FIND_LIBRARY(darwin_lib_LIBRARY
    NAMES darwin.a
    PATHS ${DARWIN_LIB_PATH}/Linux/lib)

IF (darwin_comm_INCLUDE_DIR AND darwin_linux_INCLUDE_DIR AND darwin_lib_LIBRARY)
   SET(darwin_lib_FOUND TRUE)
ENDIF (darwin_comm_INCLUDE_DIR AND darwin_linux_INCLUDE_DIR AND darwin_lib_LIBRARY)

IF (darwin_lib_FOUND)
   IF (NOT darwin_lib_FIND_QUIETLY)
      MESSAGE(STATUS "Found darwin_lib: ${darwin_lib_LIBRARY}")
      SET(darwin_lib_INCLUDE_DIR ${darwin_comm_INCLUDE_DIR} ${darwin_linux_INCLUDE_DIR})
   ENDIF (NOT darwin_lib_FIND_QUIETLY)
ELSE (darwin_lib_FOUND)
   IF (darwin_lib_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find darwin_lib")
   ENDIF (darwin_lib_FIND_REQUIRED)
ENDIF (darwin_lib_FOUND)

