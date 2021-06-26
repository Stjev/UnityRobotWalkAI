using UnityEditor;
using UnityEngine;

namespace Assets.Scripts
{
    public class RaySensors : MonoBehaviour
    {
        public static readonly int RAY_COUNT = 15;
        private static readonly int RAY_LENGTH = 5;

        private Vector3[] rayDirs;
        private int layerMask;

        public void Start()
        {
            // Don't hit (other) players or goals
            layerMask = ~LayerMask.GetMask("Player") & ~LayerMask.GetMask("Goal");

            rayDirs = new Vector3[RAY_COUNT];

            // Make an array of directions to cast the raycasts in (3 rows, 5 columns)
            for (int row = -1, i = 0; row <= 1; ++row)
            {
                for(int col = -2; col <= 2; ++col, ++i)
                {
                    // 0.4 radians spacing between columns
                    // 0.35 radians spacing between rows (with a 0.2 radian offset)
                    rayDirs[i] = new Vector3(col * 0.4f, row * 0.35f - 0.2f, 0);
                }
            }
        }

        /**
         * Get an array of floats representing the distances of each ray
         */
        public float[] GetRayDistances()
        {
            float[] raydists = new float[RAY_COUNT];
            int i = 0;

            foreach (Vector3 dir in this.rayDirs)
            {

                RaycastHit hit;
                // Does the ray intersect any objects excluding the player layer
                if (Physics.Raycast(transform.position, transform.TransformDirection(Vector3.forward + dir), out hit, RAY_LENGTH, layerMask))
                { // The ray hit
                    raydists[i] = hit.distance;

                    // Debug.DrawRay(transform.position, transform.TransformDirection(Vector3.forward + dir) * hit.distance, Color.yellow);
                }
                else
                { // The ray didn't hit
                    raydists[i] = -1;

                    // Debug.DrawRay(transform.position, transform.TransformDirection(Vector3.forward + dir) * RAY_LENGTH, Color.white);
                }
                ++i;
            }

            return raydists;
        }
    }
}

// https://docs.unity3d.com/ScriptReference/Physics.Raycast.html