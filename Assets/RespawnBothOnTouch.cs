using UnityEngine;

[RequireComponent(typeof(Collider2D))]
public class RespawnBothOnContact : MonoBehaviour
{
    [Header("Referências")]
    public Rigidbody2D runnerRb;   // fugitivo
    public Rigidbody2D chaserRb;   // perseguidor
    public Collider2D  runnerCol;  // collider do fugitivo
    public Collider2D  chaserCol;  // collider do perseguidor
    public BoxCollider2D respawnArea; // retângulo interno jogável (IsTrigger ok)

    [Header("Spawn seguro")]
    public LayerMask bloqueios;       // Parede + Obstacle
    public float raioRunner = 0.25f;  // raio de segurança do runner
    public float raioChaser = 0.25f;  // raio de segurança do chaser
    public float minSeparacao = 1.2f; // distância mínima entre eles ao nascer
    public int   tentativas = 60;

    [Header("Detecção de toque")]
    public float touchMargin = 0.02f; // tolerância extra (<= 0 usa só overlap)

    [Header("Cooldown")]
    public float cooldownSeg = 0.25f; // evita reset duplo muito rápido
    static float _nextReset = -999f;

    [Header("Round por tempo")]
    public float roundMaxSeg = 12f;   // se passar esse tempo sem contato -> respawn
    float _roundStart;

    void Start()
    {
        _roundStart = Time.time;
    }

    void FixedUpdate()
    {
        if (Time.time < _nextReset) return;
        if (!runnerRb || !chaserRb || !runnerCol || !chaserCol || !respawnArea) return;

        // 1) contato entre colliders => respawn
        ColliderDistance2D d = Physics2D.Distance(runnerCol, chaserCol);
        bool tocou = d.isOverlapped || d.distance <= touchMargin;
        if (tocou)
        {
            RespawnAmbos(); // isso já reinicia o relógio do round
            return;
        }

        // 2) estourou o tempo de round => respawn
        if (roundMaxSeg > 0f && (Time.time - _roundStart) >= roundMaxSeg)
        {
            RespawnAmbos();
            return;
        }
    }

    void RespawnAmbos()
    {
        Bounds b = respawnArea.bounds;

        for (int i = 0; i < tentativas; i++)
        {
            Vector2 pr = PontoAleatorio(b);
            if (Physics2D.OverlapCircle(pr, raioRunner, bloqueios)) continue;

            for (int j = 0; j < tentativas; j++)
            {
                Vector2 pc = PontoAleatorio(b);
                if (Physics2D.OverlapCircle(pc, raioChaser, bloqueios)) continue;
                if ((pr - pc).sqrMagnitude < minSeparacao * minSeparacao) continue;

                runnerRb.position = pr;
                chaserRb.position = pc;
                runnerRb.linearVelocity = Vector2.zero;
                chaserRb.linearVelocity = Vector2.zero;

                _nextReset = Time.time + cooldownSeg;
                _roundStart = Time.time; // reinicia o round aqui
                return;
            }
        }

        // Fallback: centro com separação mínima
        Vector2 c = b.center;
        float half = minSeparacao * 0.5f;
        runnerRb.position = c + new Vector2(-half, 0f);
        chaserRb.position = c + new Vector2(+half, 0f);
        runnerRb.linearVelocity = Vector2.zero;
        chaserRb.linearVelocity = Vector2.zero;

        _nextReset = Time.time + cooldownSeg;
        _roundStart = Time.time; // reinicia aqui também
    }

    static Vector2 PontoAleatorio(Bounds b)
    {
        return new Vector2(
            Random.Range(b.min.x, b.max.x),
            Random.Range(b.min.y, b.max.y)
        );
    }
}
