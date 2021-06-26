using Assets.Scripts;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BackCollisionDetection : MonoBehaviour
{
    private IPlayer player; 

    // Start is called before the first frame update
    void Start()
    {
        this.player = this.gameObject.GetComponentInParent<IPlayer>();
    }

    public void OnTriggerEnter(Collider other)
    {
        this.player.Die();
    }
}
