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
    }

    // Update is called once per frame
    void Update()
    {
        transform.position = new Vector3(target.position.x - distance, target.position.y, target.position.z - distance);
    }
}

// https://blog.unity.com/technology/ml-agents-v20-release-now-supports-training-complex-cooperative-behaviors