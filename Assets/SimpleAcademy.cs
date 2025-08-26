using UnityEngine;

public class SimpleAcademy : MonoBehaviour
{

        public NPCReinforcementAgent agent;


    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
                agent.OnEpisodeBegin();

    }

}
