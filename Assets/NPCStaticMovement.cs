using Pathfinding;
using UnityEngine;

[RequireComponent(typeof(Seeker), typeof(Rigidbody2D))]
public class NPCPathfinding : MonoBehaviour
{
    public Transform target;
    public float speed = 2f;                 // velocidade alvo
    public float nextWaypointDistance = 0.55f; // tolerância base

    [Header("Detecção (obstáculos internos)")]
    [SerializeField] LayerMask obstacleMask;   // mesas/caixas/etc.
    [SerializeField] string runnerTag = "Fugitivo";

    [Header("Parede/Desvio (bordas externas)")]
    public LayerMask wallMask;                 // SOMENTE as bordas/parede da arena
    [Range(0.2f, 2.5f)] public float wallProbeLen = 1.2f;     // alcance base sondas
    [Range(0.1f, 0.8f)] public float wallProbeRadius = 0.32f; // raio das sondas

    [Header("Anti-colar na parede (sem BoxCollider de limites)")]
    [Range(0.2f, 1.2f)] public float margemParede = 0.6f;     // distância mínima aceitável às bordas
    [Range(0f, 1f)]     public float pesoAlinhamento = 0.25f; // alinhamento com alvo
    [Range(0f, 1.5f)]   public float pesoInterior = 0.6f;     // empurrar p/ dentro se perto da parede
    [Range(6, 40)]      public int probesInterior = 12;       // quantas direções amostrar p/ “interior”

    [Header("Amostragem de direção segura")]
    [Range(5, 41)] public int amostras = 13;                  // ímpar (ex.: 13)
    [Range(30f, 150f)] public float lequeGraus = 90f;         // abertura total do leque

    [Header("Steering/Suavização")]
    public float acceleration = 18f;
    public float turnBlend = 0.35f;
    public float maxSpeedCap = 6f;

    [Header("Anti-travamento")]
    public float stuckSpeed = 0.05f;
    public float stuckTime = 0.6f;
    public float wanderDuration = 0.8f;
    public float wanderStrength = 1.0f;
    public bool  repathOnStuck = true;
    public float repathJitterRadius = 1.5f;

    private Path path;
    private int currentWaypoint = 0;
    private Seeker seeker;
    private Rigidbody2D rb;

    private float lowSpeedTimer = 0f;
    private float wanderUntil = 0f;
    private Vector2 wanderDir = Vector2.zero;

    void Start()
    {
        seeker = GetComponent<Seeker>();
        rb = GetComponent<Rigidbody2D>();
        rb.linearVelocity = Vector2.zero;
        rb.collisionDetectionMode = CollisionDetectionMode2D.Continuous;

        InvokeRepeating(nameof(UpdatePath), 0f, 0.25f);
    }

    void UpdatePath()
    {
        if (target == null) return;
        if (seeker.IsDone())
            seeker.StartPath(rb.position, target.position, OnPathComplete);
    }

    void OnPathComplete(Path p)
    {
        if (!p.error)
        {
            path = p;
            currentWaypoint = 0;
        }
        else path = null;
    }

    void FixedUpdate()
    {
        if (target == null) { rb.linearVelocity = Vector2.zero; return; }

        // ===== detecção de “travado” =====
        float v = rb.linearVelocity.magnitude;
        if (v < stuckSpeed) lowSpeedTimer += Time.fixedDeltaTime;
        else lowSpeedTimer = 0f;

        bool isWandering = Time.time < wanderUntil;
        if (!isWandering && lowSpeedTimer >= stuckTime)
        {
            wanderDir = Random.insideUnitCircle.normalized;
            wanderUntil = Time.time + wanderDuration;

            if (repathOnStuck && seeker.IsDone())
            {
                Vector2 jitter = Random.insideUnitCircle * repathJitterRadius;
                seeker.StartPath(rb.position, (Vector2)target.position + jitter, OnPathComplete);
            }
            lowSpeedTimer = 0f;
        }

        // ===== movimento durante wander =====
        if (isWandering)
        {
            Vector2 preferIntW = PreferenciaInteriorViaWallMask(rb.position);
            Vector2 dirEscolhidaW = MelhorDirecaoLivre(rb.position,
                                                       wanderDir,
                                                       wanderDir,
                                                       preferIntW);
            dirEscolhidaW = FugaSegura(rb.position, dirEscolhidaW);
            SteerTowards(dirEscolhidaW * speed, speed);
            return;
        }

        // ===== direção desejada (path -> alvo -> tangente) =====
        Vector2 desiredDir = ComputeDesiredDirection();

        // ===== escolhe direção segura (leque + margem + interior via wallMask) =====
        Vector2 preferInt  = PreferenciaInteriorViaWallMask(rb.position);
        Vector2 dirEscolhida = MelhorDirecaoLivre(rb.position,
                                                  desiredDir,     // base
                                                  desiredDir,     // preferir apontar pro alvo
                                                  preferInt);     // e fugir da borda se perto

        // cinto de segurança (se ainda aponta pra borda próxima, desliza tangente)
        dirEscolhida = FugaSegura(rb.position, dirEscolhida);

        // ===== steering suave =====
        Vector2 desiredVel = dirEscolhida * speed;
        SteerTowards(desiredVel, speed);
    }

    // Decide a direção “bruta” usando path/waypoints e fallback
    private Vector2 ComputeDesiredDirection()
    {
        if (path != null && currentWaypoint < path.vectorPath.Count)
        {
            Vector2 waypoint = (Vector2)path.vectorPath[currentWaypoint];
            Vector2 toWp = waypoint - rb.position;
            float dist = toWp.magnitude;
            if (dist > 1e-4f)
            {
                Vector2 dir = toWp / dist;

                // se tiver algo entre você e o waypoint, tente recalcular p/ alvo
                RaycastHit2D hit = Physics2D.CircleCast(rb.position, 0.35f, dir, dist, obstacleMask);
                if (hit.collider != null && seeker.IsDone())
                    seeker.StartPath(rb.position, target.position, OnPathComplete);

                // tolerância dinâmica
                if (dist < AdjustedWaypointTolerance(dir)) currentWaypoint++;
                return dir;
            }
        }

        // fallback: linha de visão pro alvo
        Vector2 toTarget = (Vector2)target.position - rb.position;
        float dAlvo = toTarget.magnitude;
        if (dAlvo <= 0.01f) return Vector2.zero;

        Vector2 dirAlvo = toTarget / dAlvo;
        RaycastHit2D vis = Physics2D.CircleCast(rb.position, 0.35f, dirAlvo, dAlvo, obstacleMask);

        if (vis.collider == null) return dirAlvo;

        // sem visão: desliza pela tangente
        Vector2 tang = Vector2.Perpendicular(vis.normal).normalized;
        Vector2 candA = tang;
        Vector2 candB = -tang;
        Vector2 melhor = (Vector2.Dot(candA, dirAlvo) > Vector2.Dot(candB, dirAlvo)) ? candA : candB;

        if (seeker.IsDone()) seeker.StartPath(rb.position, target.position, OnPathComplete);
        return melhor;
    }

    private float AdjustedWaypointTolerance(Vector2 dirToWp)
    {
        if (path == null || currentWaypoint + 1 >= path.vectorPath.Count) return nextWaypointDistance;
        Vector2 cur = dirToWp;
        Vector2 next = ((Vector2)path.vectorPath[currentWaypoint + 1] - (Vector2)path.vectorPath[currentWaypoint]).normalized;
        float alinh = Mathf.Clamp01(Vector2.Dot(cur, next)); // 1 bom, 0 ruim
        float extra = Mathf.Lerp(0.35f, 0f, alinh);           // +35cm quando ângulo é ruim
        return nextWaypointDistance + extra;
    }

    // ===================== anti-parede (sem Box de limites) =====================

    // “Empurrar pra dentro” calculado pelas paredes mais próximas via normals
    Vector2 PreferenciaInteriorViaWallMask(Vector2 pos)
    {
        if (wallMask.value == 0 || probesInterior <= 0) return Vector2.zero;

        float alcance = Mathf.Max(wallProbeLen, margemParede + 0.2f);
        float raio = Mathf.Max(0.05f, wallProbeRadius);

        Vector2 sum = Vector2.zero;
        int hits = 0;

        // amostra em círculo
        float step = 360f / probesInterior;
        for (int i = 0; i < probesInterior; i++)
        {
            float ang = step * i * Mathf.Deg2Rad;
            Vector2 dir = new Vector2(Mathf.Cos(ang), Mathf.Sin(ang));

            RaycastHit2D hit = Physics2D.CircleCast(pos, raio, dir, alcance, wallMask);
            if (!hit) continue;

            // distância efetiva considerando margem
            float dist = hit.distance;
            float w = Mathf.Clamp01(1f - (dist / margemParede)); // >0 se estiver dentro da margem

            if (w > 0f)
            {
                // normal aponta PARA fora da parede; empurrão pro lado oposto da parede = +normal
                Vector2 n = hit.normal.normalized;
                sum += n * w;
                hits++;
            }
        }

        if (sum.sqrMagnitude < 1e-6f) return Vector2.zero;
        return sum.normalized; // direção para “centro” prático (longe das paredes detectadas)
    }

    // Escolhe a melhor direção dentro de um leque ao redor de dirBase
    Vector2 MelhorDirecaoLivre(Vector2 origem, Vector2 dirBase, Vector2 preferAlvo, Vector2 preferDentro)
    {
        if (amostras < 5) amostras = 5;
        if ((amostras % 2) == 0) amostras++;

        float best = float.NegativeInfinity;
        Vector2 bestDir = dirBase;

        Avaliar(dirBase.normalized, 0f);

        int lados = (amostras - 1) / 2;
        float passo = lequeGraus / Mathf.Max(1, lados);

        for (int i = 1; i <= lados; i++)
        {
            float ang = passo * i;
            Avaliar(Rotate(dirBase,  ang).normalized,  ang);
            Avaliar(Rotate(dirBase, -ang).normalized, -ang);
        }

        return bestDir.normalized;

        void Avaliar(Vector2 d, float ang)
        {
            float alcance = Mathf.Max(1f, wallProbeLen);
            float livre = DistLivreComMargem(origem, d, alcance, wallMask, margemParede);
            if (livre <= 0.001f) return;

            float alinhAlvo   = Mathf.Max(0f, Vector2.Dot(d, preferAlvo));   // 0..1
            float alinhDentro = Mathf.Max(0f, Vector2.Dot(d, preferDentro)); // 0..1

            float score = livre
                        + pesoAlinhamento * alcance * alinhAlvo
                        + pesoInterior    * alcance * alinhDentro
                        - 0.01f * Mathf.Abs(ang);

            if (score > best) { best = score; bestDir = d; }
        }
    }

    // cinto de segurança: se ainda aponta pra parede dentro da margem, desliza pela tangente mais livre
    Vector2 FugaSegura(Vector2 origem, Vector2 dir)
    {
        if (wallMask.value == 0) return dir;

        float raio = Mathf.Max(0.05f, margemParede * 0.5f);
        float alcance = Mathf.Max(wallProbeLen, margemParede + 0.2f);

        RaycastHit2D hit = Physics2D.CircleCast(origem, raio, dir, alcance, wallMask);
        if (!hit) return dir;
        if (hit.distance > margemParede) return dir;

        Vector2 n  = hit.normal.normalized;
        Vector2 t1 = new Vector2(-n.y, n.x).normalized;
        Vector2 t2 = -t1;

        float d1 = DistLivreComMargem(origem, t1, alcance, wallMask, margemParede);
        float d2 = DistLivreComMargem(origem, t2, alcance, wallMask, margemParede);

        return (d1 >= d2 ? t1 : t2);
    }

    // distância “segura” até a parede, descontando a margem (mínimo 0)
    static float DistLivreComMargem(Vector2 origem, Vector2 dir, float alcance, LayerMask paredes, float margem)
    {
        if (paredes.value == 0) return alcance;

        float raio = Mathf.Max(0.05f, margem * 0.5f);
        RaycastHit2D hit = Physics2D.CircleCast(origem, raio, dir, alcance, paredes);
        if (!hit) return alcance;

        return Mathf.Max(0f, hit.distance - margem);
    }

    // util: girar vetor (graus)
    static Vector2 Rotate(Vector2 v, float graus)
    {
        float r = graus * Mathf.Deg2Rad; float c = Mathf.Cos(r), s = Mathf.Sin(r);
        return new Vector2(c*v.x - s*v.y, s*v.x + c*v.y);
    }

    private void SteerTowards(Vector2 desiredVel, float targetSpeed)
    {
        Vector2 v = rb.linearVelocity;
        Vector2 next = Vector2.MoveTowards(v, desiredVel, acceleration * Time.fixedDeltaTime);
        if (next.magnitude > Mathf.Max(targetSpeed, 0.1f))
            next = next.normalized * targetSpeed;
        rb.linearVelocity = Vector2.ClampMagnitude(next, maxSpeedCap);
    }

    private void OnTriggerEnter2D(Collider2D other)
    {
        if (!other.CompareTag(runnerTag)) return;
        var agent = other.GetComponent<NPCReinforcementAgent>();
        if (agent != null) agent.EndEpisode();
    }
}
