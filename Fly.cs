using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class Fly : MonoBehaviour
{
    [SerializeField] private GameObject head, rightHand;

    [SerializeField] private float flySpeed = 0.8f;
    private bool isFlying = false;

    XRIDefaultInputActions playerControls;

    private void Awake()
    {
        playerControls = new XRIDefaultInputActions();
    }

    #region Inputsystem

    /// <summary>
    /// Subscribes event when behaviour is enabled
    /// </summary>
    private void OnEnable()
    {
        playerControls.Enable();
    }

    /// <summary>
    /// Unsubscribes event when behaviour is disabled
    /// </summary>
    private void OnDisable()
    {
        playerControls.Disable();
    }

    void Update()
    {
        CheckIfFlying();
        FlyIfFlying();
    }

    #endregion

    #region Flying
    /// <summary>
    /// Check if player is flying to enable/disable flight ability
    /// </summary>
    private void CheckIfFlying()
    {
        //If grip and trigger are pushed down, isFlying = true
        bool _gripTriggerDown = playerControls.XRIRightHandInteraction.ActivateValue.ReadValue<float>() > 0.1f;

        if (_gripTriggerDown)
        {
            isFlying = true;
        }
    }

    /// <summary>
    /// If flycheck has passed, give ability to fly
    /// </summary>
    private void FlyIfFlying()
    {
        if (isFlying == true)
        {
            Vector3 flyDir = rightHand.transform.position - head.transform.position;
            transform.position += flyDir.normalized * flySpeed;
        }
    }
    #endregion

}
