using System;
using UnityEditor;
using UnityEngine;

namespace Assets.Scripts
{
    public static class LevelControl
    {
        public static class Level1
        {
            private static System.Random r = new System.Random();

            /**
             * This code should execute at the start of the level
             */ 
            public static void OnStart(Transform Target, Transform Spawnpoint)
            {
                // Put the goal in an initial random location
                Target.position = Level1.RandomGoal(Spawnpoint.position);
            }

            /**
             * This code should execute at the start of an episode
             */
            public static void OnEpisodeBegin(Renderer bodyRenderer)
            {
                // Set the color to red before the first goal is collected
                bodyRenderer.material.SetColor("_Color", Color.red);
            }

            /**
             * This code should execute when the Agents reached the goal
             */
            public static void OnGoalReached(Transform Target, Transform Spawnpoint, Renderer bodyRenderer, Color hideColor)
            {
                // Put the goal in a new random location
                Target.position = Level1.RandomGoal(Spawnpoint.position);

                // Set the color to red before the first goal is collected
                bodyRenderer.material.SetColor("_Color", hideColor);
            }

            /**
             * Place the Target (aka goal) at a random location around the origin
             * 
             * The origin is expected to be the spawn position of the Agent
             */
            public static Vector3 RandomGoal(Vector3 origin, int squareWidth=3)
            {
                // Generate a random position between 0 to N
                int n = r.Next(squareWidth * squareWidth - 1);

                // If this index is greater than or equal to the middlepoint, add 1
                if (n >= Math.Pow(Math.Ceiling(squareWidth / 2.0f), 2))
                {
                    n += 1;
                }

                int floorHalf = (int) Math.Floor(squareWidth / 2.0);

                // Calculate a new random location
                int row = (int) Math.Floor((float)n / squareWidth) - floorHalf;
                int col = (n % squareWidth) - floorHalf;
                Vector3 newPos = new Vector3(col, -origin.y + 1, row) + origin;

                // Add the offset of the origin and return
                return newPos;
            }
        }
    
        public static class Level2
        {

            public static void OnStart()
            {
                // Does nothing
            }

            /**
             * This code should be executed when an episode begins in level 2
             */
            public static void OnEpisodeBegin(Transform Target, Vector3 originalTargetPos)
            {
                // Reset the target position
                Target.position = originalTargetPos;
            }

            /**
             * This code should execute when the Agents reached the goal
             */
            public static void OnGoalReached(Transform Target, Transform Spawnpoint, Vector3 originalTargetPos)
            {
                if (Target.position.Equals(originalTargetPos))
                { // The target is in it's original location
                    Target.position = Spawnpoint.position - new Vector3(0, 0.5f, 0);
                } else
                { // The target is in the spawn position
                    Target.position = originalTargetPos;
                }
            }
        }
    }
}