1;

pkg load control;
pkg load symbolic;

##**************************************************************************
##*                OCTAVE PROGRAMMING (e-Yantra)
##*                ====================================
##*  This software is intended to teach Octave Programming and Mathematical
##*  Modeling concepts
##*  Theme: Lunar Scout
##*  Filename: Task_1A.m
##*  Version: 1.0.0  
##*  Date: 18/09/2023
##*
##*  Team ID :
##*  Team Leader Name:
##*  Team Member Name
##*
##*  
##*  Author: e-Yantra Project, Department of Computer Science
##*  and Engineering, Indian Institute of Technology Bombay.
##*  
##*  Software released under Creative Commons CC BY-NC-SA
##*
##*  For legal information refer to:
##*        http://creativecommons.org/licenses/by-nc-sa/4.0/legalcode 
##*     
##*
##*  This software is made available on an �AS IS WHERE IS BASIS�. 
##*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
##*  any and all claim(s) that emanate from the use of the Software or 
##*  breach of the terms of this agreement.
##*  
##*  e-Yantra - An MHRD project under National Mission on Education using 
##*  ICT(NMEICT)
##*
##**************************************************************************

## Function : Jacobian_A_B()
## ----------------------------------------------------
## Input:   Mp                - mass of the pendulum
##          l                 - Length of Pendulum
##          g                 - Acceleration due to gravity
##          Ma                - mass of the arm
##          Rp                - length of pendulum base from the pivot point
##          Ra                 - length from arm's center of mass to arm's pivot point
##          I_arm             - Moment of inertia of the arm in yaw angle
##          I_pendulum_theta  - Moment of inertia of the pendulum in tilt angle
##          I_pendulum_alpha  - Moment of inertia of the pendulum in yaw angle
##
## Output:  A - A matrix of system (State or System Matrix )
##          B - B matrix of system (Input Matrix)
##          
## Purpose: Use jacobian function to find A and B matrices(State Space Model) in this function.

function [A,B] = Jacobian_A_B(Mp,l,g,Ma,Rp,Ra,I_arm,I_pendulum_theta,I_pendulum_alpha)

  alpha = sym('alpha');
  theta = sym('theta');
  theta_dot = sym('theta_dot');
  alpha_dot = sym('alpha_dot');
  u = sym('u');
  
  cos_theta = cos(theta);
  sin_theta = sin(theta);

  cos_alpha = cos(alpha);
  sin_alpha = sin(alpha);
  
  ########## ADD YOUR CODE HERE ################
  #{
  Steps : 
    1. Define equations of motion (4-states so 4 equations). It is suggested to use Lagrangian method. You can try Newtonian methods too.
    2. Partial Differentiation of equations of motion to find the Jacobian matrices
    3. Linearization by substituting equillibrium point condition in Jacobians
  
  ### NOTE ### : Sequence of states should not be altered for evaluation
  
  SEQUENCE OF STATES : [alpha_dot; alpha; theta_dot; theta]
        Example:
                A = [x x x x;   # corresponds to ...alpha_dot
                     x x x x;   # ...alpha
                     x x x x;   # ...theta_dot
                     x x x x]   # ...theta
                B = [x;   # ...alpha_dot
                     x;   # ...alpha
                     x;   # ...theta_dot
                     x]   # ...theta  
  #}
  A = [0 1 0 0;
       -g*(Ma+Mp)/Ra*Ma 0,Mp*g/Ma*Ra,0;
       0 0 0 1;
       -g*(Ma+Mp)/l*Ra*Ma,0,-(Mp*g)/(Ma*Ra*l)+(g/l),0];
  B = [0; 
       1/(Ma*Ra^2); 
       0; 
       -1/(Ma*l*Ra)];
       
  ##############################################  
  A = double(A); # A should be (double) datatype 
  B = double(B); # B should be (double) datatype
  
endfunction

## Function : lqr_Rotary_Inverted_Pendulum()
## ----------------------------------------------------
## Input:   A - A matrix of system (State or System Matrix )
##          B - B matrix of system (Input Matrix)
##
## Output:  K - LQR Gain Matrix
##          
## Purpose: This function is used for finding optimal gains for the system using
##          the Linear Quadratic Regulator technique           

function K = lqr_Rotary_Inverted_Pendulum(A,B)
  C    =  [1 0 0 0;
          0 1 0 0;
          0 0 1 0;
          0 0 0 1];           ## Initialise C matrix
  D     = [0;0;0;0];          ## Initialise D matrix
  Q     = eye(4);             ## Initialise Q matrix
  R     = 1;                  ## Initialise R 
  sys   = ss(A,B,C,D);        ## State Space Model
  K     = lqr(sys,Q,R);       ## Calculate K matrix from A,B,Q,R matrices using lqr()
  
endfunction

## Function : Rotary_Inverted_Pendulum_main()
## ----------------------------------------------------
## Purpose: Used for testing out the various controllers by calling their 
##          respective functions.
##          (1) Tilt angle is represented as theta
##          (2) Yaw angle is represented as alpha
        
function Rotary_Inverted_Pendulum_main()
  
  Mp = 0.5 ;                  # mass of the pendulum (Kg)
  l = 0.15 ;                  # length from pendulum's center of mass to pendulum's base/pivot (meter)
  g = 9.81 ;                  # Accelertion due to gravity (kgm/s^2)
  Ma = 0.25 ;                 # mass of the arm (kg)
 
  r_a = 0.02;                 # radius of arm cylinder (meter)
  r_p = 0.02;                 # radius of pendulum cylinder (meter)
 
  Rp = 0.1 ;                  # length from pendulum's base to arm's pivot point (meter)
  Ra = 0.1 ;                   # length from arm's center of mass to arm's pivot point (meter)
  
  I_arm = 0.003;                   # Moment of inertia of the arm in yaw angle i.e. alpha (kgm^2)
  I_pendulum_theta = 0.01;        # Moment of inertia of the pendulum in tilt angle i.e. theta (kgm^2)
  I_pendulum_alpha = 0.01;        # Moment of inertia of the pendulum in yaw angle (kgm^2)
  
  [A,B] = Jacobian_A_B(Mp,l,g,Ma,Rp,Ra,I_arm,I_pendulum_theta,I_pendulum_alpha) ## find A , B matrix using  Jacobian_A_B() function
  K = lqr_Rotary_Inverted_Pendulum(A,B)  ## find the gains using lqr_Rotary_Inverted_Pendulum() function
  
endfunction
