cmake_minimum_required(VERSION 3.9)
project(BareApp)

set(CMAKE_CXX_STANDARD 11)

add_executable(BareApp
        inc/3dof.h
        inc/blink.h
        inc/dynamics.h
        inc/dynamics_3dof.h
        inc/face_lift.h
        inc/geometry.h
        3dof.c
        blink.c
        dynamics_3dof.c
        face_lift.c
        geometry.c
        main.c
        )
