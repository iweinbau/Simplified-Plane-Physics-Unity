using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Drone : MonoBehaviour {

    public Transform leftwing;
    public Transform rightwing;
    public Transform verticalStab;
    public Transform horizontalStab;

    private float gravity = 9.81f; // m/s^2
    public float wingX = 2f; // m
    public float tailSize = 1f; // m
    public float engineMass = 0.125f; // kg
    public float wingMass = 0.125f; // kg
    public float tailMass = 0.0625f; // kg
    public float maxThrust = 5f; // N
    public float maxAOA = Mathf.Deg2Rad * 15f; // radian
    public float wingLiftSlope = 1f; // radian
    public float horStabLiftSlope = 0.5f; // radian
    public float verStabLiftSlope = 0.5f; // radian

    [Range(-Mathf.PI/2, Mathf.PI / 2)]
    public float leftWingInclination;
    [Range(-Mathf.PI / 2, Mathf.PI / 2)]
    public float rightWingInclination;
    [Range(-Mathf.PI / 2, Mathf.PI / 2)]
    public float horStabInclination;
    [Range(-Mathf.PI / 2, Mathf.PI / 2)]
    public float verStabInclination;
    [Range(0, 2000)]
    public float thrust;
    private float mass;

    private Vector3 velocity = new Vector3(0,0,-50f);
    private Vector3 angularAcc = Vector3.zero;
    private Vector3 angularV =  Vector3.zero;

    private Vector3 totalForce;
    private Vector3 totalTorque;

    private Vector3 inertiaTensor;

    void Awake()
    {      
        Vector3[] relAirfoilPositions = {
        new Vector3(-wingX, 0.0f, 0.0f),
        new Vector3(wingX, 0.0f, 0.0f),
        new Vector3(0.0f, 0.0f, tailSize)};
        float[] masses = {
        wingMass,
        wingMass,
        tailMass};

        float Ixx = 0;
        float Iyy = 0;
        float Izz = 0;
        for (int i = 0; i < relAirfoilPositions.Length; i++)
        {

            Ixx += masses[i] * (Mathf.Pow(relAirfoilPositions[i].y, 2) + Mathf
                .Pow(relAirfoilPositions[i].z, 2));
            Iyy += masses[i] * (Mathf.Pow(relAirfoilPositions[i].z, 2) + Mathf
                .Pow(relAirfoilPositions[i].x, 2));
            Izz += masses[i] * (Mathf.Pow(relAirfoilPositions[i].x, 2) + Mathf
                .Pow(relAirfoilPositions[i].y, 2));
        }
        this.inertiaTensor = new Vector3(Ixx, Iyy, Izz);
        this.mass = getMass();
    }

    public float getMass()
    {
        float mass = 0;
        mass += wingMass * 2;
        mass += tailMass;
        mass += engineMass;
        return mass;
    }

    // Use this for initialization
    void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {
        leftwing.transform.localRotation = Quaternion.Euler(new Vector3(Mathf.Rad2Deg * leftWingInclination, 0, 0));
        rightwing.transform.localRotation = Quaternion.Euler(new Vector3(Mathf.Rad2Deg * rightWingInclination, 0, 0));

        verticalStab.transform.localRotation = Quaternion.Euler(new Vector3(0f,Mathf.Rad2Deg * verStabInclination, 90f));
        horizontalStab.transform.localRotation = Quaternion.Euler(new Vector3(Mathf.Rad2Deg * horStabInclination,0f,0f));

    }

    //fixed Update for physics
    void FixedUpdate()
    {
        float h = Time.fixedDeltaTime;
        //set force in drone axis
        this.totalForce = transform.TransformVector(getTotalForce());
        //set torque
        this.totalTorque = getTotalTorque();
        //setPosition
        transform.position += this.velocity * h;
        //setVelocity
        this.velocity += this.totalForce * (h / mass);
        
        transform.rotation = Quaternion.Euler(transform.rotation.eulerAngles + (this.angularV * h));
        //angularAcc
        Vector3 inertiaMatrixInv = new Vector3(1 / inertiaTensor.x, 1 / inertiaTensor.y, 1 / inertiaTensor.z);
        //calculate angularAcc
        this.angularAcc = new Vector3(inertiaMatrixInv.x * this.totalTorque.x, inertiaMatrixInv.y * this.totalTorque.y, inertiaMatrixInv.z * this.totalTorque.z);
        //angularV
        this.angularV += angularAcc * h;
    }

    /**
     * Get the total force on the drone in the drone axis.
     *
     * @return total force of the Drone in the drone axis.
     */
    Vector3 getTotalForce()
    {
        Vector3[] relAirfoilPositions = {
        new Vector3(-wingX, 0.0f, 0.0f),
        new Vector3(wingX, 0.0f, 0.0f),
        new Vector3(0.0f, 0.0f, tailSize),
        new Vector3(0.0f, 0.0f, tailSize)};

        Vector3[] axes = {
        new Vector3(1.0f, 0.0f, 0.0f),
        new Vector3(1.0f, 0.0f, 0.0f),
        new Vector3(1.0f, 0.0f, 0.0f),
        new Vector3(0.0f, 1.0f, 0.0f)};

        Vector3[] attackVectors = {
        new Vector3(0.0f, Mathf.Sin(this.leftWingInclination), -Mathf.Cos(this.leftWingInclination)),
        new Vector3(0.0f, Mathf.Sin(this.rightWingInclination),
            -Mathf.Cos(this.rightWingInclination)),
        new Vector3(0.0f, Mathf.Sin(this.horStabInclination), -Mathf.Cos(this.horStabInclination)),
        new Vector3(-Mathf.Sin(this.verStabInclination), 0.0f, -Mathf.Cos(this.verStabInclination))};

        float[] liftSlopes = {
        wingLiftSlope,
        wingLiftSlope,
        horStabLiftSlope,
        verStabLiftSlope};

        Vector3 force = new Vector3(0, 0, -thrust) +
            transform.InverseTransformVector(new Vector3(0f,this.mass * - gravity,0f));

        for (int i = 0; i < liftSlopes.Length; i++)
        {
            Vector3 axis = axes[i];
            Vector3 attackVector = attackVectors[i];
            Vector3 resPos = relAirfoilPositions[i];
            float liftSlope = liftSlopes[i];
            force += lift(resPos, axis, attackVector, liftSlope);
        }

        return force;
    }

    /**
     * the total torque in the of the drone.
     *
     * @return the total torque of the drone.
     */
    Vector3 getTotalTorque()
    {
        Vector3[] relAirfoilPositions = {
        new Vector3(-wingX, 0.0f, 0.0f),
        new Vector3(wingX, 0.0f, 0.0f),
        new Vector3(0.0f, 0.0f, tailSize),
        new Vector3(0.0f, 0.0f, tailSize)};

        Vector3[] axes = {
        new Vector3(1.0f, 0.0f, 0.0f),
        new Vector3(1.0f, 0.0f, 0.0f),
        new Vector3(1.0f, 0.0f, 0.0f),
        new Vector3(0.0f, 1.0f, 0.0f)};

        Vector3[] attackVectors = {
        new Vector3(0.0f, Mathf.Sin(this.leftWingInclination), -Mathf.Cos(this.leftWingInclination)),
        new Vector3(0.0f, Mathf.Sin(this.rightWingInclination),
            -Mathf.Cos(this.rightWingInclination)),
        new Vector3(0.0f, Mathf.Sin(this.horStabInclination), -Mathf.Cos(this.horStabInclination)),
        new Vector3(-Mathf.Sin(this.verStabInclination), 0.0f, -Mathf.Cos(this.verStabInclination))};

        float[] liftSlopes = {
        wingLiftSlope,
        wingLiftSlope,
        horStabLiftSlope,
        verStabLiftSlope};

        Vector3 totalTorque = new Vector3(0, 0, 0);

        for (int i = 0; i < liftSlopes.Length; i++)
        {
            Vector3 resPos = relAirfoilPositions[i];
            Vector3 axis = axes[i];
            Vector3 attackVector = attackVectors[i];
            float liftSlope = liftSlopes[i];
            totalTorque += Vector3.Cross(resPos, lift(resPos, axis, attackVector, liftSlope));
        }
        return totalTorque;
    }

    Vector3 lift(Vector3 relPos, Vector3 axis, Vector3 attackVector, float liftSlope)
    {
        Vector3 lift;
        Vector3 normal = Vector3.Cross(axis, attackVector);
        Vector3 projectedAirSpeed = orthogonalize(Vector3.Cross(this.angularV, relPos)
            + (transform.InverseTransformVector(this.velocity)), axis);
        float AOA = -Mathf.Atan2(Vector3.Dot(projectedAirSpeed, normal),
            Vector3.Dot(projectedAirSpeed, attackVector));
        lift = normal * (AOA * liftSlope * Mathf.Pow(projectedAirSpeed.magnitude, 2));
        return lift;
    }

    public static Vector3 orthogonalize(Vector3 v, Vector3 reff)
    {
        if (reff.magnitude > 1.0E-6f) {
            reff = reff * (1.0f / reff.magnitude);
        }
        v = v - (reff * (Vector3.Dot(v, reff)));
        return v;
    }
}
