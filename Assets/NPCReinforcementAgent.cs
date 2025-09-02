using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine;
using UnityEngine.InputSystem;

public class NPCReinforcementAgent : Agent
{
    public Transform target;
    public float moveSpeed = 3f;

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

    private Rigidbody2D rb;
    private float prevDistance, bestDistance;
    private int stepsSinceBest;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody2D>();
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

        sensor.AddObservation(agentPos);   // 2
        sensor.AddObservation(targetPos);  // 2
        sensor.AddObservation(vel);        // 2
        // total = 6
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // ação contínua (x,y) ∈ [-1,1]
        float moveX = actions.ContinuousActions[0];
        float moveY = actions.ContinuousActions[1];

        Vector2 dir = new Vector2(moveX, moveY);
        if (dir.sqrMagnitude > 1f) dir.Normalize();

        // movimento por física
        rb.linearVelocity = dir * moveSpeed;

        // distância e progresso
        float dist = Vector2.Distance(transform.localPosition, target.localPosition);
        float progress = prevDistance - dist;

        // shaping de progresso (aproximar é bom)
        AddReward(0.05f * progress);

        // passo custa (mantém urgência)
        AddReward(-0.005f);

        // penaliza ficar quase parado
        if (rb.linearVelocity.magnitude < 0.05f)
            AddReward(-0.003f);

        // sucesso
        if (dist < successRadius)
        {
            AddReward(+1.0f);
            EndEpisode();
            return;
        }

        // fora dos limites → termina
        if (Mathf.Abs(transform.localPosition.x) > mapLimitX + 0.1f ||
            Mathf.Abs(transform.localPosition.y) > mapLimitY + 0.1f)
        {
            AddReward(-0.2f);
            EndEpisode();
            return;
        }

        // early stop por estagnação
        if (dist + minProgressDelta < bestDistance)
        {
            bestDistance = dist;
            stepsSinceBest = 0;
        }
        else
        {
            stepsSinceBest++;
            if (stepsSinceBest >= noProgressWindow)
            {
                AddReward(-0.1f); // empacado
                EndEpisode();
                return;
            }
        }

        prevDistance = dist;
    }

    // punição leve ao encostar em obstáculo (defina obstacleMask)
    void OnCollisionStay2D(Collision2D col)
    {
        if (((1 << col.gameObject.layer) & obstacleMask) != 0)
        {
            AddReward(-0.002f);
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
}
