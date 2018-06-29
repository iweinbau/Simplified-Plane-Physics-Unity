using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class follow : MonoBehaviour {

    public GameObject toFollow;
    public Vector3 offsetp;
    public Vector3 offsetr;

	// Use this for initialization
	void Start () {
	}
	
	// Update is called once per frame
	void Update () {
        this.transform.position = toFollow.transform.position + offsetp;
        this.transform.rotation = Quaternion.Euler(offsetr);
	}
}
