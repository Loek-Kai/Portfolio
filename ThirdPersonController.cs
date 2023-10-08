using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class ThirdPersonController : MonoBehaviour, IEntity
{
    //input fields
    protected InputActionAsset inputAsset;
    protected InputActionMap player;
    protected InputAction move;

    //movement fields
    private Rigidbody rb;
    [SerializeField]
    private float movementForce = 1f;
    [SerializeField]
    private float jumpForce = 5f;
    [SerializeField]
    private float maxSpeed = 5f;
    private Vector3 forceDirection = Vector3.zero;

    //Animation fields
    [SerializeField]
    private Camera playerCamera;
    private Animator animator;

    [SerializeField] private Sword sword;

    private void Awake()
    {
        //Find components in object
        rb = this.GetComponent<Rigidbody>();
        animator = this.GetComponentInChildren<Animator>();
        inputAsset = this.GetComponentInParent<PlayerInput>().actions;
        sword = GetComponentInChildren<Sword>();

        player = inputAsset.FindActionMap("Player");
    }

    private void FixedUpdate()
    {
        //Add force to player camera when player moves
        forceDirection += move.ReadValue<Vector2>().x * GetCameraRight(playerCamera) * movementForce;
        forceDirection += move.ReadValue<Vector2>().y * GetCameraForward(playerCamera) * movementForce;

        //Apply force to rigidbody
        rb.AddForce(forceDirection, ForceMode.Impulse);
        //Reset forceDirection when input stops
        forceDirection = Vector3.zero;

        //Accelerate fall 
        if (rb.velocity.y < 0f)
            rb.velocity -= Vector3.down * Physics.gravity.y * Time.fixedDeltaTime;

        //Cap velocity when maxSpeed reached  
        Vector3 horizontalVelocity = rb.velocity;
        horizontalVelocity.y = 0;
        if (horizontalVelocity.sqrMagnitude > maxSpeed * maxSpeed)
            rb.velocity = horizontalVelocity.normalized * maxSpeed + Vector3.up * rb.velocity.y;

        LookAt();
    }

    private void Update()
    {
        //Gets current state of animator. If animator is playing the attack animation, disable collider
        if (animator.GetCurrentAnimatorStateInfo(1).IsName("RightHand@Attack01") && animator.GetCurrentAnimatorStateInfo(1).normalizedTime < .6f)
        {
            sword.GetComponent<MeshCollider>().enabled = true;
        }
        else
        {
            sword.GetComponent<MeshCollider>().enabled = false;
        }
    }
}
    
