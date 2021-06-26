using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraTracking : MonoBehaviour
{
    [SerializeField] private Transform target;
    [SerializeField] float distance;

    // Start is called before the first frame update
    void Start()
    {
        this.distance = -2;
    }

    // Update is called once per frame
    void Update()
    {
        transform.position = new Vector3(target.position.x - distance, transform.position.y, target.position.z - distance);
    }
}