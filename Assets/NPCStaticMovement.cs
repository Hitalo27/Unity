using Pathfinding;
using UnityEngine;

[RequireComponent(typeof(Seeker), typeof(Rigidbody2D))]
public class NPCPathfinding : MonoBehaviour
{
    public Transform target;
    public float speed = 2f;
    public float nextWaypointDistance = 0.5f;

    [Header("Detecção")]
    [SerializeField] LayerMask obstacleMask;
    [SerializeField] string runnerTag = "Fugitivo";

    [Header("Anti-travamento")]
    [Tooltip("Abaixo dessa velocidade considera travado")]
    public float stuckSpeed = 0.05f;
    [Tooltip("Tempo mínimo com baixa velocidade para acionar o anti-trava")]
    public float stuckTime = 0.6f;
    [Tooltip("Duração do desvio aleatório quando travado")]
    public float wanderDuration = 0.8f;
    [Tooltip("Força do desvio aleatório")]
    public float wanderStrength = 1.0f;
    [Tooltip("Recalcular caminho imediatamente ao detectar travamento")]
    public bool repathOnStuck = true;
    [Tooltip("Raio para escolher um ponto próximo ao alvo ao repath forçado")]
    public float repathJitterRadius = 1.5f;

    private Path path;
    private int currentWaypoint = 0;
    private Seeker seeker;
    private Rigidbody2D rb;

    // controle de travamento
    private float lowSpeedTimer = 0f;
    private float wanderUntil = 0f;
    private Vector2 wanderDir = Vector2.zero;

    void Start()
    {
        seeker = GetComponent<Seeker>();
        rb = GetComponent<Rigidbody2D>();
        rb.linearVelocity = Vector2.zero;
        rb.collisionDetectionMode = CollisionDetectionMode2D.Continuous;

        // atualizar caminho com boa frequência
        InvokeRepeating(nameof(UpdatePath), 0f, 0.2f);
    }

    void UpdatePath()
    {
        if (target == null) return;

        if (seeker.IsDone())
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
            // Path falhou: liberar para fallback em FixedUpdate
            path = null;
        }
    }

    void FixedUpdate()
    {
        if (target == null)
        {
            rb.linearVelocity = Vector2.zero;
            return;
        }

        // medir velocidade atual
        float v = rb.linearVelocity.magnitude;

        // cronômetro de baixa velocidade
        if (v < stuckSpeed)
            lowSpeedTimer += Time.fixedDeltaTime;
        else
            lowSpeedTimer = 0f;

        bool isWandering = Time.time < wanderUntil;

        // aciona anti-trava
        if (!isWandering && lowSpeedTimer >= stuckTime)
        {
            // escolhe direção aleatória e janela de wander
            wanderDir = Random.insideUnitCircle.normalized;
            wanderUntil = Time.time + wanderDuration;

            // tenta forçar novo path com leve ruído no destino
            if (repathOnStuck && seeker.IsDone())
            {
                Vector2 jitter = Random.insideUnitCircle * repathJitterRadius;
                Vector2 dest = (Vector2)target.position + jitter;
                seeker.StartPath(rb.position, dest, OnPathComplete);
            }
            // zera o timer para não rearmar a cada frame
            lowSpeedTimer = 0f;
        }

        // se estiver em wander, aplica desvio simples e sai
        if (isWandering)
        {
            rb.linearVelocity = Vector2.Lerp(rb.linearVelocity, wanderDir * (speed * wanderStrength), 0.4f);
            return;
        }

        // caminho válido
        if (path != null && currentWaypoint < path.vectorPath.Count)
        {
            Vector2 waypoint = (Vector2)path.vectorPath[currentWaypoint];
            Vector2 direction = (waypoint - rb.position).normalized;

            // se há obstáculo até o waypoint, força repath direto ao alvo
            float dist = Vector2.Distance(rb.position, waypoint);
            RaycastHit2D hit = Physics2D.CircleCast(rb.position, 0.35f, direction, dist, obstacleMask);

            if (hit.collider != null && seeker.IsDone())
            {
                seeker.StartPath(rb.position, target.position, OnPathComplete);
            }

            rb.linearVelocity = direction * speed;

            // avança waypoint
            if (Vector2.Distance(rb.position, waypoint) < nextWaypointDistance)
                currentWaypoint++;
            return;
        }

        // fallback: sem path ou terminou a lista de pontos
        // 1) se linha de visão ao alvo estiver livre, vai direto
        Vector2 toTarget = (Vector2)target.position - rb.position;
        float dAlvo = toTarget.magnitude;
        if (dAlvo > 0.01f)
        {
            Vector2 dirAlvo = toTarget / dAlvo;
            RaycastHit2D vis = Physics2D.CircleCast(rb.position, 0.35f, dirAlvo, dAlvo, obstacleMask);

            if (vis.collider == null)
            {
                rb.linearVelocity = dirAlvo * speed;
            }
            else
            {
                // 2) sem visão: tente deslizar pela normal do obstáculo enquanto repath roda
                Vector2 tangente = Vector2.Perpendicular(vis.normal).normalized;
                // escolhe o lado que mais aproxima do alvo
                Vector2 candA = tangente;
                Vector2 candB = -tangente;
                Vector2 melhor = (Vector2.Dot(candA, dirArredondado(dirAlvo)) > Vector2.Dot(candB, dirArredondado(dirAlvo))) ? candA : candB;
                rb.linearVelocity = melhor * (speed * 0.8f);

                // força um novo path se possível
                if (seeker.IsDone())
                    seeker.StartPath(rb.position, target.position, OnPathComplete);
            }
        }
        else
        {
            rb.linearVelocity = Vector2.zero;
        }
    }

    // ajuda a evitar NaN ao comparar
    private Vector2 dirArredondado(Vector2 v)
    {
        if (v.sqrMagnitude < 1e-6f) return Vector2.zero;
        return v.normalized;
    }

    void OnTriggerEnter2D(Collider2D other)
    {
        if (!other.CompareTag(runnerTag)) return;

        var agent = other.GetComponent<NPCReinforcementAgent>();
        if (agent != null)
        {
            agent.EndEpisode();
        }
    }
}
