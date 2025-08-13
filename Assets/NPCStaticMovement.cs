using Pathfinding;
using UnityEngine;

[RequireComponent(typeof(Seeker), typeof(Rigidbody2D))]
public class NPCPathfinding : MonoBehaviour
{
    public Transform target;
    public float speed = 2f;
    public float nextWaypointDistance = 0.5f;

    private Path path;
    private int currentWaypoint = 0;
    private Seeker seeker;
    private Rigidbody2D rb;

    void Start()
    {
        seeker = GetComponent<Seeker>();
        rb = GetComponent<Rigidbody2D>();
        rb.linearVelocity = Vector2.zero; // Corrigido

        // Atualiza o caminho mais frequentemente para melhorar desvio
        InvokeRepeating(nameof(UpdatePath), 0f, 0.2f);
    }

    void UpdatePath()
    {
        if (seeker.IsDone() && target != null)
        {
            seeker.StartPath(rb.position, target.position, OnPathComplete);
        }
    }

    void OnPathComplete(Path p)
    {
        if (!p.error)
        {
            path = p;
            currentWaypoint = 0;
        }
        else
        {
            Debug.LogError("Erro ao calcular o caminho!");
        }
    }

    void FixedUpdate()
    {
        if (path == null || currentWaypoint >= path.vectorPath.Count) return;

        Vector2 direction = ((Vector2)path.vectorPath[currentWaypoint] - rb.position).normalized;

        rb.linearVelocity = direction * speed; // Corrigido

        // Avança para o próximo waypoint
        if (Vector2.Distance(rb.position, path.vectorPath[currentWaypoint]) < nextWaypointDistance)
        {
            currentWaypoint++;
        }
        else
        {
            // Detecta obstáculo e recalcula caminho
            float distanceToWaypoint = Vector2.Distance(rb.position, path.vectorPath[currentWaypoint]);
            RaycastHit2D hit = Physics2D.CircleCast(
                rb.position, 
                0.5f, 
                direction, 
                distanceToWaypoint, 
                LayerMask.GetMask("Obstacles")
            );

            if (hit.collider != null)
            {
                seeker.StartPath(rb.position, target.position, OnPathComplete);
            }
        }
    }
}
