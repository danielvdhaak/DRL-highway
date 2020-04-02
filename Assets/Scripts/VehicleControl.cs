/*
 * This code is part of DRL-Highway by Daniel (2020)
 * 
 */

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace CarBehaviour {
    [RequireComponent(typeof(Rigidbody))]

    public class VehicleControl : MonoBehaviour {
        [Header("Properties")]
        public float throttleValue = 0.0f;
        public float brakingValue = 0.0f;
        public float steeringAngle = 0.0f;
        public float velocity = 0.0f;

        //public float Steering { get{ return steering; } set{ steering = Mathf.Clamp(value, -1f, 1f); } } 
        //public float Throttle { get{ return throttle; } set{ throttle = Mathf.Clamp(value, -1f, 1f); } } 

        // Use this to read the current car speed (you'll need this to make a speedometer)
        //[SerializeField] float speed = 0.0f;
        //public float Speed { get { return speed; } }

        [Header("Inputs")]
        [Tooltip("Set to true if player will control this vehicle.")]
        public bool isPlayer = false; 
        [SerializeField] string engineInput = "Vertical";
        [SerializeField] string steeringInput = "Horizontal";
        [Tooltip("Recommended to leave as default unless one desires nonlinear steering behavior")] 
        [SerializeField] AnimationCurve turnInputCurve = AnimationCurve.Linear(-1.0f, -1.0f, 1.0f, 1.0f);
        [Tooltip("GameObject center of mass")]
        [SerializeField] Transform centerOfMass;

        [Header("Wheels")]
        [SerializeField] GameObject frontLeftWheel;
        [SerializeField] GameObject frontRightWheel;
        [SerializeField] GameObject backLeftWheel;
        [SerializeField] GameObject backRightWheel;

        [Header("Behaviour")]
        [Tooltip("Curve starts at x=0 and y>0, then x=topspeed and y=0 and finally x>topspeed and y<0")]
        [SerializeField] AnimationCurve motorTorque = new AnimationCurve(new Keyframe(0, 200), new Keyframe(50, 300), new Keyframe(200, 0));
        [Range(2, 16)]
        [SerializeField] float diffGearing = 4.0f;
        [SerializeField] float brakeForce = 1500.0f;
        [SerializeField] bool enableAckermannSteering = true;
        [Range(0f, 50.0f)]
        [SerializeField] float maxSteerAngle = 30.0f;
        [Range(0.001f, 1.0f)]
        [SerializeField] float steerSpeed = 0.6f;
        [Range(0.5f, 10f)]
        [SerializeField] float downforce = 0.0f;


        // All other private variables
        float steering, engine;
        float l, w;

        Rigidbody rb;

        GameObject[] allWheels;

        WheelCollider frontLeftWheelCol;
        WheelCollider frontRightWheelCol;
        WheelCollider backLeftWheelCol;
        WheelCollider backRightWheelCol;
        WheelCollider[] allWheelCols;
        WheelCollider[] turnWheelCols;
        WheelCollider[] driveWheelCols;

        VehicleWheels vehicleWheels;


    void Start() {
            // Initialize rigid body and center of mass
            rb = GetComponent<Rigidbody>();
            if (rb != null && centerOfMass != null)
            {
                rb.centerOfMass = centerOfMass.localPosition;
            }

            allWheels = new GameObject[] { frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel };

            // Initialize wheelcolliders
            frontLeftWheelCol = frontLeftWheel.GetComponent<WheelCollider>();
            frontRightWheelCol = frontRightWheel.GetComponent<WheelCollider>();
            backLeftWheelCol = backLeftWheel.GetComponent<WheelCollider>();
            backRightWheelCol = backRightWheel.GetComponent<WheelCollider>();
            allWheelCols = new WheelCollider[] { frontLeftWheelCol, frontRightWheelCol, backLeftWheelCol, backRightWheelCol };
            turnWheelCols = new WheelCollider[] { frontLeftWheelCol, frontRightWheelCol };
            driveWheelCols = new WheelCollider[] { backLeftWheelCol, backRightWheelCol };

            // Calculate wheel seperation w and base l
            w = Math.Abs(frontLeftWheelCol.transform.position.x - frontRightWheelCol.transform.position.x);
            l = Math.Abs(frontLeftWheelCol.transform.position.z - backLeftWheelCol.transform.position.z);


            // Initialize wheel turning class
            vehicleWheels = GetComponent<VehicleWheels>();
        }

        void GetSpeed()
        {
            // Receives vehicle velocity in km/h
            velocity = transform.InverseTransformDirection(rb.velocity).z * 3.6f;
        }

        void GetPlayerInput()
        {
            // Obtain steering input
            steering = Input.GetAxis(steeringInput);

            // Obtain engine input
            engine = Input.GetAxis(engineInput);
        }

        void TurnWheels ()
        {
            // Use Ackermann steering or regular steering depending on configuration
            if (enableAckermannSteering)
            {
                double angle = steeringAngle * Mathf.Deg2Rad;
                frontLeftWheelCol.steerAngle = (float)Math.Atan((2 * l * Math.Sin(angle)) / (2 * l * Math.Cos(angle) + w * Math.Sin(angle))) * Mathf.Rad2Deg;
                frontRightWheelCol.steerAngle = (float)Math.Atan((2 * l * Math.Sin(angle)) / (2 * l * Math.Cos(angle) - w * Math.Sin(angle))) * Mathf.Rad2Deg;
            }
            else
            {
                foreach(WheelCollider wheel in turnWheelCols)
                {
                    wheel.steerAngle = steeringAngle;
                }
            }

        }

        void Accelerate ()
        {
            // Remove previous brake torques
            foreach (WheelCollider wheel in allWheelCols)
            {
                wheel.brakeTorque = 0;
            }

            // EITHER velocity and throttle are consistent OR velocity is significantly slow enough: acceleration
            if (Math.Sign(velocity) == Math.Sign(throttleValue) || Math.Abs(velocity) < 2) 
            {
                foreach (WheelCollider wheel in driveWheelCols)
                {
                    wheel.motorTorque = throttleValue * motorTorque.Evaluate(velocity) * diffGearing / 2;
                }
            }
            // If velocity and throttle are opposite: braking
            else if (Math.Sign(velocity) != Math.Sign(throttleValue))
            {
                foreach(WheelCollider wheel in allWheelCols)
                {
                    wheel.motorTorque = 0;
                    wheel.brakeTorque = Math.Abs(throttleValue) * brakeForce;
                }
            }
        }
        
        void FixedUpdate () {
            GetSpeed();

            if (isPlayer) {
                GetPlayerInput();
            }

            // Set values (needs rework for external setting!)
            steeringAngle = turnInputCurve.Evaluate(steering) * maxSteerAngle;
            throttleValue = engine;

            // Turn and accelerate
            TurnWheels();
            Accelerate();
            
            // Downforce
            rb.AddForce(-transform.up * velocity * downforce);
        }

        private void Update()
        {
            // Turn wheels if VehicleWheels script is attached
            if (vehicleWheels != null)
            {
                foreach (GameObject wheel in allWheels)
                {
                    vehicleWheels.TurnWheel(wheel);
                }

            }

            // Update lights
            // lightfunction
        }
    }
}
