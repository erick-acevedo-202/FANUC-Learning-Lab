using UnityEngine;

public class StartupLagLogger : MonoBehaviour
{
    public Rigidbody rb;
    public float logSeconds = 1.0f;
    float t;

    void Reset() { rb = GetComponent<Rigidbody>(); }

    void Awake()
    {
        Debug.Log($"[Diag] Awake t={Time.realtimeSinceStartup:F3}");
    }

    void Start()
    {
        Debug.Log($"[Diag] Start t={Time.realtimeSinceStartup:F3} " +
                  $"maxDelta={Time.maximumDeltaTime:F3} fixedDelta={Time.fixedDeltaTime:F3}");
        if (rb != null)
        {
            Debug.Log($"[Diag] RB present. isKinematic={rb.isKinematic} " +
                      $"interp={rb.interpolation} CDM={rb.collisionDetectionMode}");
        }
        else
        {
            Debug.Log("[Diag] ¡No hay Rigidbody asignado!");
        }
    }

    void Update()
    {
        t += Time.unscaledDeltaTime;
        if (t <= logSeconds)
        {
            bool sleeping = rb ? rb.IsSleeping() : false;
            float vel = rb ? rb.velocity.magnitude : 0f;
            Debug.Log($"[Diag] Frame={Time.frameCount} dt={Time.deltaTime:F3} " +
                      $"udt={Time.unscaledDeltaTime:F3} sleeping={sleeping} vel={vel:F3}");
        }
    }

    void FixedUpdate()
    {
        bool sleeping = rb ? rb.IsSleeping() : false;
        Debug.Log($"[Diag] Fixed frame={Time.frameCount} fdt={Time.fixedDeltaTime:F3} sleeping={sleeping}");
    }
}
