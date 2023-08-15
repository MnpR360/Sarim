using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VehicleController : MonoBehaviour
{
    // jjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjjj
    // Motor torque values container
    private MotorTorqueValues motorTorque = new MotorTorqueValues();

    // Wheel colliders for left and right wheels
    public WheelCollider[] leftWheelColliders;
    public WheelCollider[] rightWheelColliders;

    // Visual wheels game objects
    public GameObject[] leftVisualWheels;
    public GameObject[] rightVisualWheels;

    // Vehicle parameters
    public float motorForce = 1000f;
    public float speedLimit = 10.0f;
    public float maxSteerAngle = 30f;
    public float differentialFactor = 0.5f; // Adjust this value to control differential steering
    public float rotatePower = 2f;




    private void Start()
    {


    }

    private void FixedUpdate()
    {
        // Read user input
        float verticalInput = Input.GetAxis("Vertical"); // W/S or Up/Down arrow keys
        float horizontalInput = Input.GetAxis("Horizontal"); // A/D or Left/Right arrow keys

        // Apply motor force
        ApplyMotorForce(verticalInput);


         ApplySteering(horizontalInput);
 
    

        // Update visual wheels
        UpdateVisualWheels();
    }

    // Apply motor force based on speed limit
    void ApplyMotorForce(float input)
    {
        float currentSpeed = Mathf.Abs(leftWheelColliders[0].rpm *
            (2.0f * Mathf.PI * leftWheelColliders[0].radius) / 60.0f);

        if (Mathf.Abs(currentSpeed) < speedLimit)
        {
            foreach (WheelCollider wheelCollider in leftWheelColliders)
            {
                wheelCollider.motorTorque = input * motorForce;// + motorTorque.leftTorque;
            }
            foreach (WheelCollider wheelCollider in rightWheelColliders)
            {
                wheelCollider.motorTorque = input * motorForce + motorTorque.rightTorque;
            }
        }
        else
        {
            foreach (WheelCollider wheelCollider in rightWheelColliders)
            {
                wheelCollider.motorTorque = 0;
            }
        }
    }

    // Apply steering torque
    void ApplySteering(float dir)
    {
        float leftTorque = dir * motorForce * rotatePower;
        float rightTorque = -dir * motorForce * rotatePower;

        foreach (WheelCollider wheelCollider in leftWheelColliders)
        {
            motorTorque.leftTorque = leftTorque;
        }
        foreach (WheelCollider wheelCollider in rightWheelColliders)
        {
            motorTorque.rightTorque = rightTorque;
        }
    }

    // Update visual wheels' positions and rotations
    void UpdateVisualWheels()
    {
        UpdateVisualWheels(leftWheelColliders, leftVisualWheels);
        UpdateVisualWheels(rightWheelColliders, rightVisualWheels);
    }

    // Helper method to update visual wheels
    void UpdateVisualWheels(WheelCollider[] wheelColliders, GameObject[] visualWheels)
    {
        for (int i = 0; i < visualWheels.Length; i++)
        {
            WheelCollider wheelCollider = wheelColliders[i];
            GameObject visualWheel = visualWheels[i];

            Vector3 wheelPosition;
            Quaternion wheelRotation;
            wheelCollider.GetWorldPose(out wheelPosition, out wheelRotation);
            visualWheel.transform.position = wheelPosition;
            visualWheel.transform.rotation = wheelRotation;
        }
    }

    // Nested class to store motor torque values
    private class MotorTorqueValues
    {
        public float leftTorque;
        public float rightTorque;
    }
}
