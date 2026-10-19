// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jack Sidman Smith
//
// Shim: lets a copy of the original uarm_coord_convert.c compile on its own (tools/kin/run.sh).
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#define RAD_TO_DEG 57.295779513082320876798154814105  /* Arduino.h:51, included first in the original */
struct { float init_arml_angle, init_armr_angle, init_base_angle; struct { float front_offset, high_offset; } param; } uarm;
bool is_angle_legal(float, float, float);
void get_current_angle(float, float, float, float*, float*, float*);
void angle_to_coord(float, float, float, float*, float*, float*);
void coord_to_angle(float, float, float, float*, float*, float*);
void get_current_step(float, float, float, float*, float*, float*);
void coord_arm2effect(float*, float*, float*);
void coord_effect2arm(float*, float*, float*);
