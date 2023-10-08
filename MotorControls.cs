using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarScript : MonoBehaviour
{
    [SerializeField] private Rigidbody sphereBody;
    [SerializeField] private Transform castFront, castBack;
    private Rigidbody modelBody;

    [Space]
    [SerializeField] private float modelOffset;
    [SerializeField] private LayerMask groundMask;
    [SerializeField] private float alignmentSpeed = 1;
    [SerializeField] private float gravityScale = 9.81f;

    [Space]
    [SerializeField] [Min(float.Epsilon)] private float mass = 1;

    [Space]
    [SerializeField] private AnimationCurve tireGripBasedOnVelocity;
    [SerializeField] private float maxTurnAngle = 5;
    [SerializeField] private AnimationCurve turnAngleBasedOnVelocity;
    [SerializeField] private float leanIntensity = 15;

    [Space]
    [SerializeField] private AnimationCurve torqueBasedOnVelocity;
    [SerializeField] private float maxSpeed = 500;
    [SerializeField] private float dragPercent = 0.1f;

    [Space]
    [SerializeField] private float bobHeight;
    [SerializeField] private float bobSpeed;
    private float bobTime = 0;
    public float MaxSpeed { get { return maxSpeed; } }

    private float forwardVelocity 
    { 
        get
        {
            float v = Vector3.Dot(currentForward, sphereBody.velocity);
            if (float.IsNaN(v))
            {
                v = 0;
            }
            return v;
        } 
    }
    public float ForwardVelocity { get { return forwardVelocity; } }
    public float NormalizedVelocity { get { return Mathf.Clamp01(Mathf.Abs(forwardVelocity) / maxSpeed); } }

    private float sideVelocity
    {
        get
        {
            float v = Vector3.Dot(modelBody.transform.right, sphereBody.velocity);
            if (float.IsNaN(v))
            {
                v = 0;
            }
            return v;
        }
    }

    private Vector2 inputAxis;

    private Vector3 currentDown;
    private Vector3 currentForward;

    private void Start()
    {
        currentDown = Vector3.down;
        modelBody = GetComponent<Rigidbody>();
    }

    private void Update()
    {
        inputAxis = new Vector2(Input.GetAxis("Horizontal"), Input.GetAxis("Vertical"));
        GetCurrentDown();
    }

    private void FixedUpdate()
    {
        HoverModelOverGround();
        RotateModelWithGround();
        Physics.gravity = currentDown * gravityScale;

        Steer();

        Drift();
        Accelerate();
        ApplyDrag();
        Debug.DrawLine(sphereBody.position, sphereBody.position + sphereBody.velocity, Color.cyan);
    }

    private void Drift()
    {
        float tireGrip = tireGripBasedOnVelocity.Evaluate(NormalizedVelocity);

        float targetVelocityChange = -sideVelocity * tireGrip;
        float targetAcceleration = targetVelocityChange / Time.fixedDeltaTime;
        sphereBody.AddForce(modelBody.transform.right * (targetAcceleration * mass), ForceMode.Acceleration);
    }

    private void Accelerate()
    {
        if (forwardVelocity < maxSpeed)
        {
            float targetAcceleration = torqueBasedOnVelocity.Evaluate(NormalizedVelocity) / Time.deltaTime;
            sphereBody.AddForce(transform.forward * (inputAxis.y * targetAcceleration * mass), ForceMode.Acceleration);
        }
    }

    private void Steer()
    {
        float rotationAngle = inputAxis.x * (turnAngleBasedOnVelocity.Evaluate(NormalizedVelocity) * maxTurnAngle) * Time.deltaTime;
        Quaternion rotationChange = Quaternion.AngleAxis(rotationAngle, Vector3.up);
        modelBody.MoveRotation(modelBody.rotation * rotationChange);
    }

    /// <summary>
    /// uses the barycentric coordinate of a raycast to determine an accurate surface normal
    /// interpolates between the three normals of a tri
    /// </summary>
    private void GetCurrentDown()
    {
        RaycastHit hit;
        if (!Physics.Raycast(sphereBody.position, currentDown, out hit, Mathf.Infinity, groundMask))
        {
            currentDown = Vector3.down;
            return;
        }

        //check for collider and if it has a sharedmesh
        MeshCollider hitMeshCollider = hit.collider as MeshCollider;
        if (hitMeshCollider == null)
        {
            Debug.LogError("hit does not have meshcollider");
            return;
        }
        if (hitMeshCollider.sharedMesh == null)
        {
            Debug.LogError("hit does not have sharedmesh");
            return;
        }

        //get the normals and triangles of the mesh
        Mesh hitMesh = hitMeshCollider.sharedMesh;
        Vector3[] meshNormals = hitMesh.normals;
        int[] meshTris = hitMesh.triangles;

        //get the normals of the 3 points of the triangle you hit
        int adjustedIndex = hit.triangleIndex * 3;
        Vector3 triNormalX = meshNormals[meshTris[adjustedIndex + 0]];
        Vector3 triNormalY = meshNormals[meshTris[adjustedIndex + 1]];
        Vector3 triNormalZ = meshNormals[meshTris[adjustedIndex + 2]];

        //the point within the triangle that you hit, expressed in a vector 3 where X + Y + Z adds up to 1
        Vector3 baryCenter = hit.barycentricCoordinate;

        //scale normals with barycenter
        //this makes it so a normal will weigh more when the hit point is closer to it
        triNormalX *= baryCenter.x;
        triNormalY *= baryCenter.y;
        triNormalZ *= baryCenter.z;

        //interpolate the normals the 3 points and normalize the result
        Vector3 interpolatedNormal = (triNormalX + triNormalY + triNormalZ).normalized;

        //convert to worldspace
        Transform hitTransform = hit.collider.transform;
        interpolatedNormal = hitTransform.TransformDirection(interpolatedNormal);

        //set current down opposite of the interpolated normal
        currentDown = -interpolatedNormal;
    }

    /// <summary>
    /// aligns rotation with the current up direction and a generated forward direction
    /// sends down two raycasts from the front and back and gets a new forward based on those points
    /// </summary>
    private void RotateModelWithGround()
    {
        currentForward = transform.forward;

        RaycastHit frontHit, backHit;
        if (Physics.Raycast(castFront.position, currentDown, out frontHit, Mathf.Infinity, groundMask) && Physics.Raycast(castBack.position, currentDown, out backHit, Mathf.Infinity, groundMask))
        {
            Vector3 targetForward = (frontHit.point - backHit.point).normalized;
            currentForward = Vector3.Lerp(currentForward, targetForward, alignmentSpeed * Time.deltaTime);
        }
        modelBody.rotation = Quaternion.LookRotation(currentForward, -currentDown);
    }

    private void HoverModelOverGround()
    {
        bobTime += Time.fixedDeltaTime;
        float bobOffset = (1 - NormalizedVelocity) * (Mathf.Sin(bobTime * bobSpeed) * bobHeight);
        Vector3 smoothPos = Vector3.Lerp(transform.position, sphereBody.position + currentDown * (modelOffset + bobOffset), alignmentSpeed * Time.deltaTime);
        modelBody.MovePosition(smoothPos);
    }

    private void LeanBikeIntoTurn()
    {
        float leanAngle = (sideVelocity * NormalizedVelocity) * leanIntensity;
        //Quaternion leanedRotation = Quaternion.AngleAxis(leanAngle, currentForward)
    }

    private void ApplyDrag()
    {
        if (Input.GetAxisRaw("Vertical") == 0)
        {
            float dragForce = (dragPercent * forwardVelocity) / Time.deltaTime;
            sphereBody.AddForce(-transform.forward * (dragForce * mass), ForceMode.Acceleration);
        }
    }
}
