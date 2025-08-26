using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine;

public class NPCReinforcementAgent : Agent
{
    public Transform target;
    public float moveSpeed = 3f;
    private Rigidbody2D rb;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody2D>();
    }

    public override void OnEpisodeBegin()
    {
        // Reseta a posição do agente e do alvo
        this.transform.localPosition = new Vector3(Random.Range(-4f, 4f), Random.Range(-4f, 4f), 0);
        //target.localPosition = new Vector3(Random.Range(-4f, 4f), Random.Range(-4f, 4f), 0);

        // Reseta a velocidade
        rb.linearVelocity = Vector2.zero;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Posição do agente
        sensor.AddObservation(this.transform.localPosition);
        // Posição do alvo
        sensor.AddObservation(target.localPosition);
        // Velocidade atual
        sensor.AddObservation(rb.linearVelocity);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Ações contínuas para movimento (horizontal e vertical)
        float moveX = actions.ContinuousActions[0];
        float moveY = actions.ContinuousActions[1];

        // Aplica o movimento baseado nas ações
        Vector2 move = new Vector2(moveX, moveY);
        rb.linearVelocity = move * moveSpeed;

        // Penalidade por tempo
        AddReward(-0.001f);

        // Recompensa quando o agente chega perto do alvo
        if (Vector2.Distance(this.transform.localPosition, target.localPosition) < 1.0f)
        {
            AddReward(1.0f);
            EndEpisode(); // Encerra o episódio quando atingir o alvo
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Permite controle manual usando o teclado
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Horizontal");
        continuousActionsOut[1] = Input.GetAxis("Vertical");
    }
}
