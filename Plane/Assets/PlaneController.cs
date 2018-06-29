using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlaneController : MonoBehaviour {

    Drone drone;

    public float horStabInclinationEffect = 0.2f;
    public float verStabInclinationEffect = 0.2f;

    // Use this for initialization
    void Start () {
        drone = GetComponent<Drone>();
	}
	
	// Update is called once per frame
	void Update () {

        if (Input.GetKey(KeyCode.UpArrow))
        {
            drone.horStabInclination += horStabInclinationEffect * Time.deltaTime;
        }

        else if (Input.GetKey(KeyCode.DownArrow))
        {
            drone.horStabInclination += -horStabInclinationEffect * Time.deltaTime;
        }

        if (Input.GetKey(KeyCode.LeftArrow))
        {
            drone.verStabInclination += verStabInclinationEffect * Time.deltaTime;
        }

        else if (Input.GetKey(KeyCode.RightArrow))
        {
            drone.verStabInclination += -verStabInclinationEffect * Time.deltaTime;
        }
    }
}
