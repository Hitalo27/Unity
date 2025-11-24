using Pathfinding;
using UnityEngine;

[RequireComponent(typeof(Seeker), typeof(Rigidbody2D))]
public class NPCPathfinding : MonoBehaviour
{
    public Transform target;
    public float speed = 2f;
    public float nextWaypointDistance = 0.55f;

    [Header("Detecção (obstáculos internos)")]
    [SerializeField] LayerMask obstacleMask;   // mesas/caixas/etc.
    [SerializeField] string runnerTag = "Fugitivo";

    [Header("Parede/Desvio (bordas externas)")]
    public LayerMask wallMask;                 // SOMENTE as bordas/parede da arena
    [Range(0.2f, 2.5f)] public float wallProbeLen = 1.6f;
    [Range(0.1f, 0.8f)] public float wallProbeRadius = 0.38f;

    [Header("Anti-colar na parede (sem BoxCollider de limites)")]
    [Range(0.2f, 1.2f)] public float margemParede = 0.8f;
    [Range(0f, 1f)]     public float pesoAlinhamento = 0.25f;
    [Range(0f, 1.5f)]   public float pesoInterior = 0.6f;
    [Range(6, 40)]      public int probesInterior = 16;

    [Header("Amostragem de direção segura")]
    [Range(5, 41)] public int amostras = 13;
    [Range(30f, 150f)] public float lequeGraus = 90f;

    [Header("Steering/Suavização")]
    public float acceleration = 18f;
    public float turnBlend = 0.35f;
    public float maxSpeedCap = 6f;

    [Header("Anti-travamento")]
    public float stuckSpeed = 0.30f;
    public float stuckTime = 0.35f;
    public float kickSpeed = 3.0f;
    public float kickDuration = 0.25f;
    public int   unstuckSamples = 13;
    public float unstuckCone = 120f;
    public float unstuckProbeLen = 1.8f;
    public float unstuckProbeRadius = 0.28f;

    [Header("Tamanho aproximado do NPC")]
    public float agentRadius = 0.35f;      // usado para "não encostar" nos obstáculos
    public float clearanceExtra = 0.10f;   // folga além do raio

    private Path path;
    private int currentWaypoint = 0;
    private Seeker seeker;
    private Rigidbody2D rb;

    private float _lowSpeedTimer = 0f;
    private float _kickUntil = -999f;
    private Vector2 _kickDir = Vector2.zero;

    void Start()
    {
        seeker = GetComponent<Seeker>();
        rb = GetComponent<Rigidbody2D>();

        rb.linearVelocity = Vector2.zero;
        rb.sleepMode = RigidbodySleepMode2D.NeverSleep;
        rb.interpolation = RigidbodyInterpolation2D.Interpolate;
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

        // chute (anti-travamento em andamento)
        if (Time.time < _kickUntil)
        {
            rb.linearVelocity = _kickDir * Mathf.Max(kickSpeed, speed);
            return;
        }

        float _spd = rb.linearVelocity.magnitude;
        float limiar = stuckSpeed;

        if (_spd < limiar) _lowSpeedTimer += Time.fixedDeltaTime;
        else _lowSpeedTimer = 0f;

        if (_lowSpeedTimer >= stuckTime)
        {
            Vector2 prefer = ComputeDesiredDirection();
            Vector2 dir    = FindUnstuckDirection(rb.position, prefer);

            _kickDir = (dir.sqrMagnitude > 1e-6f ? dir.normalized : Random.insideUnitCircle.normalized);
            _kickUntil = Time.time + kickDuration;

            rb.linearVelocity = _kickDir * Mathf.Max(kickSpeed, speed);
            _lowSpeedTimer = 0f;
            return;
        }

        // direção base (sem considerar obstáculos ainda)
        Vector2 desiredDir = ComputeDesiredDirection();

        // prefere interior com base nas paredes
        Vector2 preferInt  = PreferenciaInteriorViaWallMask(rb.position);

        // aqui entra a lógica de "não encostar": amostragem por parede + obstáculos
        Vector2 dirEscolhida = MelhorDirecaoLivre(rb.position,
                                                  desiredDir,
                                                  desiredDir,
                                                  preferInt);

        dirEscolhida = FugaSegura(rb.position, dirEscolhida);

        Vector2 desiredVel = dirEscolhida * speed;
        SteerTowards(desiredVel, speed);
    }

    // path + fallback simples (sem desvio aqui, só direção "bruta")
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
                if (dist < AdjustedWaypointTolerance(dir)) currentWaypoint++;
                return dir;
            }
        }

        Vector2 toTarget = (Vector2)target.position - rb.position;
        float dAlvo = toTarget.magnitude;
        if (dAlvo <= 0.01f) return Vector2.zero;

        return toTarget / dAlvo;
    }

    private float AdjustedWaypointTolerance(Vector2 dirToWp)
    {
        if (path == null || currentWaypoint + 1 >= path.vectorPath.Count) return nextWaypointDistance;
        Vector2 cur = dirToWp;
        Vector2 next = ((Vector2)path.vectorPath[currentWaypoint + 1] - (Vector2)path.vectorPath[currentWaypoint]).normalized;
        float alinh = Mathf.Clamp01(Vector2.Dot(cur, next));
        float extra = Mathf.Lerp(0.35f, 0f, alinh);
        return nextWaypointDistance + extra;
    }

    // ===================== paredes (empurrar para dentro) =====================

    Vector2 PreferenciaInteriorViaWallMask(Vector2 pos)
    {
        if (wallMask.value == 0 || probesInterior <= 0) return Vector2.zero;

        float alcance = Mathf.Max(wallProbeLen, margemParede + 0.2f);
        float raio = Mathf.Max(0.05f, wallProbeRadius);

        Vector2 sum = Vector2.zero;
        float step = 360f / probesInterior;

        for (int i = 0; i < probesInterior; i++)
        {
            float ang = step * i * Mathf.Deg2Rad;
            Vector2 dir = new Vector2(Mathf.Cos(ang), Mathf.Sin(ang));

            RaycastHit2D hit = Physics2D.CircleCast(pos, raio, dir, alcance, wallMask);
            if (!hit) continue;

            float dist = hit.distance;
            float w = Mathf.Clamp01(1f - (dist / margemParede));
            if (w > 0f)
            {
                Vector2 n = hit.normal.normalized;
                sum += n * w;
            }
        }

        if (sum.sqrMagnitude < 1e-6f) return Vector2.zero;
        return sum.normalized;
    }

    // DIREÇÃO SEGURA: leva em conta paredes E obstáculos para não encostar
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

        if (best <= float.NegativeInfinity + 1f)
        {
            if (preferDentro.sqrMagnitude > 1e-6f) return preferDentro.normalized;
            return Rotate(dirBase, 90f).normalized;
        }

        return bestDir.normalized;

        void Avaliar(Vector2 d, float ang)
        {
            float alcance = Mathf.Max(1f, wallProbeLen);

            // distância até parede (com margem)
            float livreParede = DistLivreComMargem(origem, d, alcance, wallMask, margemParede);

            // distância até obstáculo interno
            float distObs = DistLivreEnv(origem, d, alcance,
                                         agentRadius + clearanceExtra,
                                         obstacleMask);

            // se obstáculo estiver muito perto nessa direção, descarta
            if (distObs < agentRadius + clearanceExtra * 0.5f)
                return;

            // espaço efetivo: o menor entre parede e obstáculo
            float livre = Mathf.Min(livreParede, distObs);

            if (livre <= 0.001f) return;

            float alinhAlvo   = Mathf.Max(0f, Vector2.Dot(d, preferAlvo));
            float alinhDentro = Mathf.Max(0f, Vector2.Dot(d, preferDentro));

            float score = livre
                        + pesoAlinhamento * alcance * alinhAlvo
                        + pesoInterior    * alcance * alinhDentro
                        - 0.01f * Mathf.Abs(ang);

            if (score > best) { best = score; bestDir = d; }
        }
    }

    Vector2 FugaSegura(Vector2 origem, Vector2 dir)
    {
        if (wallMask.value == 0) return dir;

        float raio = Mathf.Max(0.05f, margemParede * 0.5f);
        float alcance = Mathf.Max(wallProbeLen, margemParede + 0.2f);

        RaycastHit2D hit = Physics2D.CircleCast(origem, raio, dir, alcance, wallMask);
        if (!hit) return dir;
        if (hit.distance > margemParede) return dir;

        Vector2 n  = hit.normal.normalized;
        Vector2 t1 = new Vector2(-n.y,  n.x).normalized;
        Vector2 t2 = -t1;

        float d1 = DistLivreComMargem(origem, t1, alcance, wallMask, margemParede);
        float d2 = DistLivreComMargem(origem, t2, alcance, wallMask, margemParede);

        if (d1 <= 0.001f && d2 <= 0.001f)
        {
            Vector2 interior = PreferenciaInteriorViaWallMask(origem);
            if (interior.sqrMagnitude > 1e-6f) return interior;
            return t1;
        }
        return (d1 >= d2 ? t1 : t2);
    }

    static float DistLivreComMargem(Vector2 origem, Vector2 dir, float alcance, LayerMask paredes, float margem)
    {
        if (paredes.value == 0) return alcance;

        float raio = Mathf.Max(0.05f, margem * 0.5f);
        RaycastHit2D hit = Physics2D.CircleCast(origem, raio, dir, alcance, paredes);
        if (!hit) return alcance;

        return Mathf.Max(0f, hit.distance - margem);
    }

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

    // ===== utilitários =====

    float DistLivreEnv(Vector2 origem, Vector2 dir, float alcance, float raio, LayerMask mask)
    {
        if (mask.value == 0) return alcance;
        RaycastHit2D hit = Physics2D.CircleCast(origem, raio, dir.normalized, alcance, mask);
        return hit ? hit.distance : alcance;
    }

    Vector2 FindUnstuckDirection(Vector2 pos, Vector2 dirPreferida)
    {
        LayerMask combined = wallMask | obstacleMask;

        Vector2 baseDir = (dirPreferida.sqrMagnitude > 1e-6f ? dirPreferida.normalized
                                                             : Random.insideUnitCircle.normalized);

        Vector2 best = baseDir;
        float bestScore = -1f;

        int n = Mathf.Max(5, unstuckSamples | 1);
        int lados = (n - 1) / 2;
        float passo = unstuckCone / Mathf.Max(1, lados);

        Vector2 toTarget = (target != null) ? ((Vector2)target.position - pos) : Vector2.zero;
        Vector2 dirAlvo = (toTarget.sqrMagnitude > 1e-6f ? toTarget.normalized : Vector2.right);

        void Avaliar(Vector2 d, float ang)
        {
            float livre = DistLivreEnv(pos, d, unstuckProbeLen,
                                       agentRadius + clearanceExtra,
                                       combined);
            if (livre <= 0.05f) return;

            float alinh = Mathf.Max(0f, Vector2.Dot(d, dirAlvo));
            float score = livre + 0.30f * unstuckProbeLen * alinh - 0.005f * Mathf.Abs(ang);

            if (score > bestScore) { bestScore = score; best = d.normalized; }
        }

        Avaliar(baseDir, 0f);
        for (int i = 1; i <= lados; i++)
        {
            float ang = passo * i;
            Avaliar(Rotate(baseDir,  ang),  ang);
            Avaliar(Rotate(baseDir, -ang), -ang);
        }

        if (bestScore < 0f) best = Random.insideUnitCircle.normalized;
        return best;
    }

    // empurrão extra em contato (seguro se algo escapar da lógica principal)
    void OnCollisionStay2D(Collision2D col)
    {
        if (rb.linearVelocity.magnitude > 0.2f) return;

        Vector2 normal = Vector2.zero;
        foreach (var c in col.contacts)
            normal += c.normal;

        if (normal.sqrMagnitude < 1e-6f) return;
        normal.Normalize();

        Vector2 tangente = new Vector2(-normal.y, normal.x);
        rb.AddForce(tangente * 2.5f, ForceMode2D.Impulse);
    }
}
