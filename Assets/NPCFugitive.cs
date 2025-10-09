using UnityEngine;

[RequireComponent(typeof(Rigidbody2D))]
public class NPCFugitive : MonoBehaviour
{
    public Transform perseguidor;
    public float moveSpeed = 1f;
    public float rayDist = 2f;
    public LayerMask obstaculos;
    public float pesoDesvio = 1.2f;
    public float ruído = 0.15f;
    public float limiteVel = 6f;

    // paredes e varredura
    public LayerMask paredes;         // SÓ a layer das bordas
    [Range(5, 41)] public int amostras = 13;
    public float lequeGraus = 90f;
    public float margemParede = 0.6f; // distância mínima aceitável
    public float pesoAlinhamento = 0.25f;

    // preferência para dentro da arena
    public BoxCollider2D limites;     // Box da arena (IsTrigger = true)
    public float pesoInterior = 0.6f; // favorece apontar para dentro quando perto

    Rigidbody2D rb;

    void Awake() => rb = GetComponent<Rigidbody2D>();

    void FixedUpdate()
    {
        if (perseguidor == null) { rb.linearVelocity = Vector2.zero; return; }

        Vector2 pos = rb.position;

        // fuga “bruta”
        Vector2 dirFuga = (pos - (Vector2)perseguidor.position);
        if (dirFuga.sqrMagnitude < 1e-6f) dirFuga = Random.insideUnitCircle;
        dirFuga.Normalize();

        // 3 antenas (obstáculos internos)
        Vector2 frente   = dirFuga;
        Vector2 esquerda = Rot(frente,  30f);
        Vector2 direita  = Rot(frente, -30f);

        Vector2 desvio = Vector2.zero;
        if (Physics2D.Raycast(pos, frente,   rayDist, obstaculos)) desvio += Perp(frente);
        if (Physics2D.Raycast(pos, esquerda, rayDist, obstaculos)) desvio += Perp(esquerda);
        if (Physics2D.Raycast(pos, direita,  rayDist, obstaculos)) desvio += Perp(direita);

        Vector2 jitter = Random.insideUnitCircle * ruído;

        Vector2 dirBase = (dirFuga + pesoDesvio * desvio + jitter).normalized;

        // preferência para dentro (0 longe da borda, 1 colado)
        Vector2 preferDentro = PreferenciaInterior(pos);

        // escolhe direção com espaço
        Vector2 dirFinal = MelhorDirecaoLivre(pos, dirBase, dirFuga, preferDentro);

        // checagem final: se ainda aponta para parede perto, gira para a tangente livre
        dirFinal = FugaSegura(pos, dirFinal);

        rb.linearVelocity = Vector2.ClampMagnitude(dirFinal * moveSpeed, limiteVel);
    }

    Vector2 MelhorDirecaoLivre(Vector2 origem, Vector2 dirBase, Vector2 preferFuga, Vector2 preferDentro)
    {
        if (amostras < 5) amostras = 5;

        float best = float.NegativeInfinity;
        Vector2 bestDir = dirBase;

        Avaliar(dirBase, 0f);

        int lados = (amostras - 1) / 2;
        float passo = lequeGraus / Mathf.Max(1, lados);

        for (int i = 1; i <= lados; i++)
        {
            float ang = passo * i;
            Avaliar(Rot(dirBase,  ang),  ang);
            Avaliar(Rot(dirBase, -ang), -ang);
        }

        return bestDir.normalized;

        void Avaliar(Vector2 d, float ang)
        {
            d.Normalize();

            // folga real até parede (com margem)
            float livre = DistLivreComMargem(origem, d, rayDist, paredes, margemParede);

            // hard stop: se não tem margem, descarta
            if (livre <= 0.001f) return;

            float alinhFuga   = Mathf.Max(0f, Vector2.Dot(d, preferFuga));   // 0..1
            float alinhDentro = Mathf.Max(0f, Vector2.Dot(d, preferDentro)); // 0..1

            // score: folga + alinhamento com fuga + alinhamento com interior - penalizar curva grande
            float score = livre
                        + pesoAlinhamento * rayDist * alinhFuga
                        + pesoInterior    * rayDist * alinhDentro
                        - 0.01f * Mathf.Abs(ang);

            if (score > best)
            {
                best = score;
                bestDir = d;
            }
        }
    }

    // corta aproximação insegura: se o ray em dir bater dentro da margem, vira para a tangente com mais folga
    Vector2 FugaSegura(Vector2 origem, Vector2 dir)
    {
        if (paredes.value == 0) return dir;

        float raio = Mathf.Max(0.05f, margemParede * 0.5f);
        float alcance = Mathf.Max(rayDist, margemParede + 0.2f);

        RaycastHit2D hit = Physics2D.CircleCast(origem, raio, dir, alcance, paredes);
        if (!hit) return dir;

        if (hit.distance > margemParede) return dir;

        Vector2 n = hit.normal.normalized;
        Vector2 t1 = new Vector2(-n.y, n.x).normalized;
        Vector2 t2 = -t1;

        float d1 = DistLivreComMargem(origem, t1, alcance, paredes, margemParede);
        float d2 = DistLivreComMargem(origem, t2, alcance, paredes, margemParede);

        return (d1 >= d2 ? t1 : t2);
    }

    // direção para o miolo do Box, cresce quando perto da borda; zero se sem limites
    Vector2 PreferenciaInterior(Vector2 pos)
    {
        if (!limites) return Vector2.zero;

        Vector2 local = limites.transform.InverseTransformPoint(pos);
        Vector2 half = limites.size * 0.5f;

        // quão perto está de cada lado
        float dxL = local.x - (-half.x);
        float dxR =  half.x - local.x;
        float dyB = local.y - (-half.y);
        float dyT =  half.y - local.y;

        float dMin = Mathf.Min(dxL, dxR, dyB, dyT);
        float f = Mathf.Clamp01(1f - dMin / margemParede); // 0 longe, 1 colado

        if (f <= 0f) return Vector2.zero;

        Vector2 alvoLocal = new Vector2(
            Mathf.Clamp(local.x, -half.x + margemParede, half.x - margemParede),
            Mathf.Clamp(local.y, -half.y + margemParede, half.y - margemParede)
        );
        Vector2 alvoMundo = limites.transform.TransformPoint(alvoLocal);
        Vector2 dir = (alvoMundo - pos);
        return dir.sqrMagnitude > 1e-6f ? dir.normalized : Vector2.zero;
    }

    // folga = distância até parede − margem (mínimo 0)
    static float DistLivreComMargem(Vector2 origem, Vector2 dir, float alcance, LayerMask paredes, float margem)
    {
        if (paredes.value == 0) return alcance;

        float raio = Mathf.Max(0.05f, margem * 0.5f);
        RaycastHit2D hit = Physics2D.CircleCast(origem, raio, dir, alcance, paredes);
        if (!hit) return alcance;

        return Mathf.Max(0f, hit.distance - margem);
    }

    static Vector2 Rot(Vector2 v, float graus)
    {
        float r = graus * Mathf.Deg2Rad; float c = Mathf.Cos(r), s = Mathf.Sin(r);
        return new Vector2(c*v.x - s*v.y, s*v.x + c*v.y);
    }

    static Vector2 Perp(Vector2 v) => new Vector2(-v.y, v.x).normalized;
}
