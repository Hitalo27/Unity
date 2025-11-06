using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine;
using UnityEngine.InputSystem;

public class NPCReinforcementAgent : Agent
{
    public Transform target;
    public float moveSpeed = 2f;

    // limites do mapa só para normalizar observações
    public float mapLimitX = 4f;
    public float mapLimitY = 4f;

    public float successRadius = 1.0f;

    // estagnação
    public int  noProgressWindow = 80;
    public float minProgressDelta = 0.01f;

    // obstáculos internos (mesas/caixas/etc.)
    public LayerMask obstacleMask;

    // desvio por “3 antenas” (obstáculos internos)
    public float rayDist = 2f;
    public float avoidAngle = 30f;
    public float avoidWeight = 1.2f;
    public float agentRadius = 0.35f;
    public float jitter = 0.0f;

    // ===== paredes externas (anti-colar na borda) via layer =====
    [Header("Parede (bordas externas)")]
    public LayerMask wallMask;                 // SOMENTE a layer das bordas
    [Range(0.2f, 1.2f)] public float margemParede = 0.6f;
    [Range(0.2f, 2.5f)] public float wallProbeLen = 1.2f;     // alcance base p/ medir folga
    [Range(0.1f, 0.8f)] public float wallProbeRadius = 0.32f; // raio dos casts

    [Header("Amostragem de direção segura")]
    [Range(5, 41)] public int amostras = 13;                  // ímpar (13/17/21)
    [Range(30f, 150f)] public float lequeGraus = 90f;         // abertura total do leque
    [Range(0f, 1f)]     public float pesoAlinhamento = 0.25f; // alinhar com alvo
    [Range(0f, 1.5f)]   public float pesoInterior = 0.6f;     // empurrar p/ “dentro” se perto
    [Range(6, 40)]      public int probesInterior = 12;       // amostras radiais p/ detectar borda

    // ===== Anti-oscilação =====
    [Header("Anti-oscilação (mínimo)")]
    public float steerSmoothing = 0.4f;
    public int   flipThreshold  = 4;
    public int   commitDuration = 12;
    public float commitStrength = 0.5f;

    // ===== Modos de comparação e currículo de velocidade =====
    [Header("Fair Play / Currículo")]
    [Tooltip("Comparação justa: sem vantagem oculta e sem assistência interior.")]
    public bool comparisonMode = true;
    [Tooltip("Durante o treino, permite pequena vantagem de velocidade e reduz até 1.0.")]
    public bool useSpeedCurriculum = true;
    [Range(1.0f, 1.5f)] public float speedStart = 1.00f;
    [Range(1.0f, 1.5f)] public float speedEnd   = 1.00f;
    public int stepsAnneal = 200_000;

    [SerializeField] string runnerTag = "Fugitivo";

    private Vector2 prevSteer = Vector2.zero;
    private int     flipStreak = 0;
    private int     commitTimer = 0;
    private Vector2 commitBias  = Vector2.zero;

    private Rigidbody2D rb;
    private float prevDistance, bestDistance;
    private int stepsSinceBest;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody2D>();
        rb.collisionDetectionMode = CollisionDetectionMode2D.Continuous;
    }

    public override void OnEpisodeBegin()
    {
        transform.localPosition = new Vector3(Random.Range(-mapLimitX, mapLimitX), Random.Range(-mapLimitY, mapLimitY), 0);
        target.localPosition    = new Vector3(Random.Range(-mapLimitX, mapLimitX), Random.Range(-mapLimitY, mapLimitY), 0);

        rb.linearVelocity = Vector2.zero;

        prevDistance  = Vector2.Distance(transform.localPosition, target.localPosition);
        bestDistance  = prevDistance;
        stepsSinceBest = 0;

        prevSteer   = Vector2.zero;
        flipStreak  = 0;
        commitTimer = 0;
        commitBias  = Vector2.zero;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Vector2 agentPos  = (Vector2)transform.localPosition;
        Vector2 targetPos = (Vector2)target.localPosition;

        Vector2 halfMap = new Vector2(Mathf.Max(mapLimitX, 0.001f),
                                      Mathf.Max(mapLimitY, 0.001f));

        Vector2 delta = targetPos - agentPos;
        Vector2 deltaNorm = new Vector2(delta.x / halfMap.x, delta.y / halfMap.y);
        sensor.AddObservation(deltaNorm); // 2

        sensor.AddObservation(rb.linearVelocity / Mathf.Max(moveSpeed, 0.001f)); // 2

        var rays = SampleAvoidRays();
        sensor.AddObservation(rays.wf);
        sensor.AddObservation(rays.wl);
        sensor.AddObservation(rays.wr); // 3
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // ação contínua (x,y) ∈ [-1,1]
        float moveX = actions.ContinuousActions[0];
        float moveY = actions.ContinuousActions[1];

        Vector2 dir = new Vector2(moveX, moveY);
        if (dir.sqrMagnitude > 1f) dir.Normalize();

        // desvio de obstáculos internos (3 antenas)
        var rays = SampleAvoidRays(dir);
        Vector2 avoid = rays.wf * Perp(rays.fwd) + rays.wl * Perp(rays.left) + rays.wr * Perp(rays.right);

        // direção “desejada” bruta (ação + desvio interno + ruído)
        Vector2 desired = dir + avoidWeight * avoid;
        if (jitter > 0f) desired += Random.insideUnitCircle * jitter;
        if (desired.sqrMagnitude > 1f) desired.Normalize();

        // ===== ANTI-PAREDE (somente wallMask) =====
        Vector2 toTarget = (target != null)
            ? ((Vector2)(target.position - transform.position)).normalized
            : Vector2.right;

        // Desliga o "empurrão interior" no modo de comparação para não dar assistência
        float pesoInteriorAtivo = comparisonMode ? 0f : pesoInterior;

        Vector2 preferDentro = PreferenciaInteriorViaWallMask(rb.position);
        Vector2 safeDir = MelhorDirecaoLivre(rb.position,
                                             desired.sqrMagnitude > 1e-6f ? desired : toTarget,
                                             toTarget,
                                             preferDentro,
                                             pesoInteriorAtivo);
        safeDir = FugaSegura(rb.position, safeDir);
        // ==========================================

        // ===== Anti-oscilação: suavizar + commit =====
        Vector2 smooth = Vector2.Lerp(prevSteer, safeDir, Mathf.Clamp01(steerSmoothing));
        if (smooth.sqrMagnitude > 1f) smooth.Normalize();

        float dot = (prevSteer.sqrMagnitude < 1e-4f) ? 1f
                   : Vector2.Dot(prevSteer.normalized, smooth.normalized);

        if (dot < -0.2f) flipStreak++;
        else if (dot > 0.4f) flipStreak = Mathf.Max(0, flipStreak - 1);

        float turnCost = 1f - Mathf.Clamp01((dot + 1f) * 0.5f); // 0..1
        if (!comparisonMode) AddReward(-0.0003f * turnCost); // desliga custo no fair play

        if (commitTimer <= 0 && flipStreak >= flipThreshold)
        {
            bool preferLeft = rays.wl < rays.wr; // lado com menos obstáculo interno
            Vector2 baseDir = (smooth.sqrMagnitude > 1e-4f) ? smooth.normalized : Vector2.right;
            commitBias  = Rotate(baseDir, preferLeft ? +45f : -45f);
            commitTimer = commitDuration;
            flipStreak  = 0;
        }

        if (commitTimer > 0)
        {
            smooth = Vector2.Lerp(smooth, commitBias, commitStrength);
            commitTimer--;
            if (!comparisonMode) AddReward(+0.0002f);
        }

        // ======== FAIR PLAY: VELOCIDADE JUSTA ========
        if (smooth.sqrMagnitude > 1e-6f) smooth.Normalize();

        // currículo de velocidade (apenas no treino, e desligado no modo de comparação)
        float speedMul = 1.0f;
        if (useSpeedCurriculum && !comparisonMode && Academy.Instance.IsCommunicatorOn)
        {
            float t = Mathf.Clamp01((float)Academy.Instance.StepCount / Mathf.Max(1, stepsAnneal));
            speedMul = Mathf.Lerp(speedStart, speedEnd, t);
        }
        float v = moveSpeed * speedMul;

        rb.linearVelocity = smooth * v;
        // clamp: em comparação, nunca ultrapassa moveSpeed; no treino, respeita v
        rb.linearVelocity = Vector2.ClampMagnitude(rb.linearVelocity, comparisonMode ? moveSpeed : v);

        // Telemetria (TensorBoard)
        Academy.Instance.StatsRecorder.Add("agent/speed", rb.linearVelocity.magnitude);
        // ==============================================

        // ===== Recompensas =====
        float dist = Vector2.Distance(transform.localPosition, target.localPosition);
        float progress = prevDistance - dist;

        AddReward(0.6f * Mathf.Clamp(progress, -0.1f, 0.1f));
        AddReward(-0.001f);

        if (!comparisonMode && progress < 0f) AddReward(-0.01f);
        if (!comparisonMode && rb.linearVelocity.magnitude < 0.05f) AddReward(-0.003f);

        float nearWall = rays.wf + rays.wl + rays.wr; // (internos) penaliza “apertado”
        if (!comparisonMode) AddReward(-0.0003f * nearWall);

        // captura opcional por raio
        // if (dist < successRadius)
        // {
        //     AddReward(+1.0f);
        //     EndEpisode();
        //     return;
        // }

        // estagnação (como você tinha)
        if (dist + minProgressDelta < bestDistance)
        {
            bestDistance = dist;
            stepsSinceBest = 0;
        }
        else
        {
            stepsSinceBest++;
            // if (stepsSinceBest >= noProgressWindow)
            // {
            //     AddReward(-0.05f);
            //     EndEpisode();
            //     return;
            // }
        }

        prevDistance = dist;
        prevSteer = smooth;
    }

    // colisão com obstáculo interno
    void OnCollisionEnter2D(Collision2D col)
    {
        if (((1 << col.gameObject.layer) & obstacleMask) != 0)
        {
            if (!comparisonMode) AddReward(-0.01f);
            if (commitTimer > 0) commitTimer = Mathf.Max(4, commitTimer / 2);
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var ca = actionsOut.ContinuousActions;
        var k = Keyboard.current;
        float x = 0f, y = 0f;
        if (k != null)
        {
            x = (k.rightArrowKey.isPressed ? 1f : 0f) + (k.leftArrowKey.isPressed ? -1f : 0f);
            y = (k.upArrowKey.isPressed    ? 1f : 0f) + (k.downArrowKey.isPressed ? -1f : 0f);
        }
        ca[0] = Mathf.Clamp(x, -1f, 1f);
        ca[1] = Mathf.Clamp(y, -1f, 1f);
    }

    void FixedUpdate()
    {
        RequestDecision();
    }

    // captura por trigger (no filho "CaptureRange")
    void OnTriggerEnter2D(Collider2D other)
    {
        if (!other.CompareTag(runnerTag)) return;
        AddReward(+1.0f);
        EndEpisode();
    }

    // ===== utilidades de desvio (obstáculos internos) =====
    (Vector2 fwd, Vector2 left, Vector2 right, float wf, float wl, float wr) SampleAvoidRays(Vector2? moveDir = null)
    {
        Vector2 baseDir = moveDir.HasValue && moveDir.Value.sqrMagnitude > 0.01f
            ? moveDir.Value.normalized
            : (target != null
                ? ((Vector2)(target.position - transform.position)).normalized
                : Vector2.right);

        Vector2 fwd   = baseDir;
        Vector2 left  = Rotate(baseDir,  +avoidAngle);
        Vector2 right = Rotate(baseDir,  -avoidAngle);

        var hf = Physics2D.CircleCast(rb.position, agentRadius, fwd,   rayDist, obstacleMask);
        var hl = Physics2D.CircleCast(rb.position, agentRadius, left,  rayDist, obstacleMask);
        var hr = Physics2D.CircleCast(rb.position, agentRadius, right, rayDist, obstacleMask);

        float wf = hf.collider ? 1f - Mathf.Clamp01(hf.distance / rayDist) : 0f;
        float wl = hl.collider ? 1f - Mathf.Clamp01(hl.distance / rayDist) : 0f;
        float wr = hr.collider ? 1f - Mathf.Clamp01(hr.distance / rayDist) : 0f;

        // debug opcional
        DebugDrawRay(rb.position, fwd,   hf, Color.white);
        DebugDrawRay(rb.position, left,  hl, Color.white);
        DebugDrawRay(rb.position, right, hr, Color.white);

        return (fwd, left, right, wf, wl, wr);
    }

    // ===================== ANTI-PAREDE (somente wallMask) =====================

    // “Empurrar pra dentro” calculado pelas paredes próximas via normals
    Vector2 PreferenciaInteriorViaWallMask(Vector2 pos)
    {
        if (wallMask.value == 0 || probesInterior <= 0) return Vector2.zero;

        float alcance = Mathf.Max(wallProbeLen, margemParede + 0.2f);
        float raio = Mathf.Max(0.05f, wallProbeRadius);

        Vector2 sum = Vector2.zero;

        float step = 360f / Mathf.Max(1, probesInterior);
        for (int i = 0; i < probesInterior; i++)
        {
            float ang = step * i * Mathf.Deg2Rad;
            Vector2 dir = new Vector2(Mathf.Cos(ang), Mathf.Sin(ang));

            RaycastHit2D hit = Physics2D.CircleCast(pos, raio, dir, alcance, wallMask);
            if (!hit) continue;

            float dist = hit.distance;
            float w = Mathf.Clamp01(1f - (dist / margemParede)); // >0 se dentro da margem
            if (w > 0f) sum += hit.normal.normalized * w;        // normal aponta p/ fora da parede
        }

        if (sum.sqrMagnitude < 1e-6f) return Vector2.zero;
        return sum.normalized; // longe das paredes detectadas
    }

    // Escolhe a melhor direção dentro de um leque ao redor de dirBase
    Vector2 MelhorDirecaoLivre(Vector2 origem, Vector2 dirBase, Vector2 preferAlvo, Vector2 preferDentro, float pesoInteriorParam)
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
            if (livre <= 0.001f) return; // sem margem -> descarta

            float alinhAlvo   = Mathf.Max(0f, Vector2.Dot(d, preferAlvo));      // 0..1
            float alinhDentro = Mathf.Max(0f, Vector2.Dot(d, preferDentro));    // 0..1

            float score = livre
                        + pesoAlinhamento * alcance * alinhAlvo
                        + pesoInteriorParam * alcance * alinhDentro // usa peso ativo (0 no fair play)
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
    // ========================================================================

    static Vector2 Rotate(Vector2 v, float degrees)
    {
        float rad = degrees * Mathf.Deg2Rad;
        float cos = Mathf.Cos(rad), sin = Mathf.Sin(rad);
        return new Vector2(v.x * cos - v.y * sin, v.x * sin + v.y * cos);
    }

    static Vector2 Perp(Vector2 v) => new Vector2(-v.y, v.x);

    static void DebugDrawRay(Vector2 origin, Vector2 dir, RaycastHit2D hit, Color c)
    {
        #if UNITY_EDITOR
        float len = hit.collider ? hit.distance : 0f;
        Debug.DrawRay(origin, dir.normalized * (hit.collider ? len : 0f), c, 0f);
        if (hit.collider) Debug.DrawRay(origin + dir.normalized * len, dir.normalized * 0.15f, Color.red, 0f);
        #endif
    }
}
