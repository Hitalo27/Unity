using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine;
using UnityEngine.InputSystem;

public class NPCReinforcementAgent : Agent
{
    public Transform target;
    public float moveSpeed = 2f;


    // limites do mapa (meia-extensão)
    public float mapLimitX = 4f;
    public float mapLimitY = 4f;

    // sucesso mais “fácil” no começo; depois pode reduzir p/ 0.6f ou 0.5f
    public float successRadius = 1.0f;

    // estagnação: se não melhorar por N steps, termina
    public int  noProgressWindow = 80;
    public float minProgressDelta = 0.01f;

    // layer das paredes/obstáculos (defina no Inspector)
    public LayerMask obstacleMask;

    public float rayDist = 2f;
    public float avoidAngle = 30f;     // graus para as “antenas” laterais
    public float avoidWeight = 1.2f;   // peso do desvio
    public float agentRadius = 0.35f;  // raio para o CircleCast
    public float jitter = 0.0f;        // ruído opcional

    // >>> NOVO: tag do fugitivo (o runner)
    [SerializeField] string runnerTag = "Fugitivo";

    
    //TODO 
    //NPC Tomar penalidade dependendo do tamanho do obstaculo para aprender a desviar função para isso e tentar desviar
    
    //robocode IA para o npc que vai correr 
    //https://github.com/robo-code/robocode

    //metodologia: movimentações estrategicas
    //atualizar o texto com oq conversamos e a diferença com oq mudei no npc fixo

    private Rigidbody2D rb;
    private float prevDistance, bestDistance;
    private int stepsSinceBest;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody2D>();
        rb.collisionDetectionMode = CollisionDetectionMode2D.Continuous; // evitar “túnel”
    }

    public override void OnEpisodeBegin()
    {
        // reposiciona agente e alvo
        transform.localPosition = new Vector3(Random.Range(-mapLimitX, mapLimitX), Random.Range(-mapLimitY, mapLimitY), 0);
        target.localPosition    = new Vector3(Random.Range(-mapLimitX, mapLimitX), Random.Range(-mapLimitY, mapLimitY), 0);

        // zera velocidade
        rb.linearVelocity = Vector2.zero;

        // inicializa distâncias
        prevDistance  = Vector2.Distance(transform.localPosition, target.localPosition);
        bestDistance  = prevDistance;
        stepsSinceBest = 0;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // >>> 2D apenas (6 floats) <<<
        Vector2 agentPos  = (Vector2)transform.localPosition;
        Vector2 targetPos = (Vector2)target.localPosition;
        Vector2 vel       = rb.linearVelocity;

    Vector2 halfMap = new Vector2(Mathf.Max(mapLimitX, 0.001f),
                                  Mathf.Max(mapLimitY, 0.001f));

    // delta normalizado [-1..1] aprox
    Vector2 delta = targetPos - agentPos;
    Vector2 deltaNorm = new Vector2(delta.x / halfMap.x, delta.y / halfMap.y);
    sensor.AddObservation(deltaNorm);                                  // 2

    // velocidade normalizada pelo moveSpeed
    sensor.AddObservation(rb.linearVelocity / Mathf.Max(moveSpeed, 0.001f)); // 2

    // “antenas” de parede (0..1)
    var rays = SampleAvoidRays();
    sensor.AddObservation(rays.wf);
    sensor.AddObservation(rays.wl);
    sensor.AddObservation(rays.wr);                                     // 3
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // ação contínua (x,y) ∈ [-1,1]
        float moveX = actions.ContinuousActions[0];
        float moveY = actions.ContinuousActions[1];

        Vector2 dir = new Vector2(moveX, moveY);
        if (dir.sqrMagnitude > 1f) dir.Normalize();

        // NOVO: desvio de obstáculos por “3 antenas” com CircleCast
         var rays = SampleAvoidRays(dir);
        Vector2 avoid = rays.wf * Perp(rays.fwd) + rays.wl * Perp(rays.left) + rays.wr * Perp(rays.right);

        Vector2 steer = dir + avoidWeight * avoid;
        if (jitter > 0f) steer += Random.insideUnitCircle * jitter;

        if (steer.sqrMagnitude > 1f) steer.Normalize();
        rb.linearVelocity = steer * moveSpeed;

        // movimento por física
        // rb.linearVelocity = dir * moveSpeed;

        // distância e progresso
        float dist = Vector2.Distance(transform.localPosition, target.localPosition);
        float progress = prevDistance - dist;

    // Progresso tem que dominar (clamp pra estabilidade)
        AddReward(0.6f * Mathf.Clamp(progress, -0.1f, 0.1f));

            // Passo custa pouco ()
            AddReward(-0.001f);

    // Aproximar-se é bom; afastar-se é ruim (sinal claro)
    if (progress < 0f) AddReward(-0.01f);

        // penaliza ficar quase parado
        if (rb.linearVelocity.magnitude < 0.05f)
            AddReward(-0.003f);

        // Pequena punição se perto de parede (ajuda a preferir caminhos limpos)
     float nearWall = rays.wf + rays.wl + rays.wr; // 0..3
        AddReward(-0.0003f * nearWall);

        if (dist < successRadius)
        {
            AddReward(+1.0f);
            EndEpisode();
            return;
        }
        

        // // fora dos limites → termina
        // if (Mathf.Abs(transform.localPosition.x) > mapLimitX + 0.1f ||
        //     Mathf.Abs(transform.localPosition.y) > mapLimitY + 0.1f)
        // {
        //     AddReward(-0.2f);
        //     EndEpisode();
        //     return;
        // }

        // early stop por estagnação
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
            //     AddReward(-0.1f); // empacado
            //     EndEpisode();
            //     return;
            // }
        }

        prevDistance = dist;
    }

        // Punição mais visível em colisão
    void OnCollisionEnter2D(Collision2D col)
    {
        if (((1 << col.gameObject.layer) & obstacleMask) != 0)
        {
            AddReward(-0.01f);
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
        // garante decisão a cada tick de física (não precisa de DecisionRequester)
        RequestDecision();
    }

// >>> NOVO: captura por trigger (no filho "CaptureRange")
    void OnTriggerEnter2D(Collider2D other)
    {
        if (!other.CompareTag(runnerTag)) return;

        // Capturou o fugitivo
        AddReward(+1.0f);  // recompensa por pegar
        EndEpisode();
        // aqui você pode notificar o jogo/placar, tocar som, etc.
    }

// ===== utilidades de desvio =====

    // Amostra 3 raios e retorna direções e pesos por proximidade 0..1
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

        // Debug opcional
        DebugDrawRay(rb.position, fwd,   hf, Color.white);
        DebugDrawRay(rb.position, left,  hl, Color.white);
        DebugDrawRay(rb.position, right, hr, Color.white);

        return (fwd, left, right, wf, wl, wr);
    }

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
        Debug.DrawRay(origin + dir.normalized * len, dir.normalized * 0.15f, Color.red, 0f);
        #endif
    }
}