using UnityEngine;
using System.IO; // para File e StreamWriter
using System.Globalization;

[RequireComponent(typeof(Collider2D))]
public class RespawnBothOnContact : MonoBehaviour
{
    [Header("Referências")]
    public Rigidbody2D runnerRb;
    public Rigidbody2D chaserRb;
    public Collider2D  runnerCol;
    public Collider2D  chaserCol;
    public BoxCollider2D respawnArea;

    [Header("Spawn seguro")]
    public LayerMask bloqueios;
    public float raioRunner = 0.25f;
    public float raioChaser = 0.25f;
    public float minSeparacao = 1.2f;
    public int tentativas = 60;

    [Header("Detecção de toque")]
    public float touchMargin = 0.02f;

    [Header("Cooldown")]
    public float cooldownSeg = 0.25f;
    static float _nextReset = -999f;

    [Header("Round por tempo")]
    public float roundMaxSeg = 30f;
    float _roundStart;

    // ===== métricas =====
    [Header("Métricas por janelas fixas (0-10, 10-20, 20-30)")]
    public int roundsPorLote = 50;// TAMBÉM será o LIMITE de linhas no CSV de eventos

    // contadores do lote atual
    int capt0_10 = 0;
    int capt10_20 = 0;
    int capt20_30 = 0;
    int escapes = 0;
    int roundsFeitosNoLote = 0;

    // contadores globais
    int roundIndexGlobal = 0;

    // arquivos CSV
    string _csvPathAgregado;
    string _csvPathEventos;

    // ===== controle de escrita de eventos =====
    int  eventosGravados = 0;   // quantas linhas já foram escritas no respawn_eventos.csv
    bool _travarEventos = false; // true = NÃO escreve mais no CSV de eventos

    [Header("Respawn em cantos opostos")]
    [Tooltip("Margem para não nascer colado na parede")]
    public float margemParede = 0.35f;

    [Tooltip("Alterna TL×BR e TR×BL a cada round")]
    public bool alternarDiagonais = true;

    [Tooltip("Se ligado, alterna quem vai para o 1º canto (runner/chaser) a cada round")]
    public bool alternarQuemFicaNoPrimeiroCanto = false;

    // estado interno de alternância
    bool _usaTL_BR = true;          // começa TL×BR
    bool _primeiroVaiRunner = true; // runner começa no 1º canto

    void Start()
    {
        _roundStart = Time.time;
        roundMaxSeg = 30f; // round fixo

        string dir = Path.Combine(Application.dataPath, "Logs");
        if (!Directory.Exists(dir)) Directory.CreateDirectory(dir);

        _csvPathAgregado = Path.Combine(dir, "respawn_agregado.csv");
        _csvPathEventos  = Path.Combine(dir, "respawn_eventos.csv");

        // cabeçalho agregado
        if (!File.Exists(_csvPathAgregado))
        {
            using (var w = new StreamWriter(_csvPathAgregado, false))
                w.WriteLine("Lote,Capt_0_10,Capt_10_20,Capt_20_30,Escapes,TotalRounds");
        }

        // cabeçalho eventos por round
        if (!File.Exists(_csvPathEventos))
        {
            using (var w = new StreamWriter(_csvPathEventos, false))
                w.WriteLine("RoundIndex,Capturado,TempoCapturaSeg,Bucket");
        }
    }

    void FixedUpdate()
    {
        if (Time.time < _nextReset) return;
        if (!runnerRb || !chaserRb || !runnerCol || !chaserCol || !respawnArea) return;

        ColliderDistance2D d = Physics2D.Distance(runnerCol, chaserCol);
        bool tocou = d.isOverlapped || d.distance <= touchMargin;

        if (tocou)
        {
            RespawnAmbos(true);
            return;
        }

        if ((Time.time - _roundStart) >= roundMaxSeg)
        {
            RespawnAmbos(false);
            return;
        }
    }

    void RespawnAmbos(bool foiPego)
    {
        // tempo decorrido no round
        float elapsed = Time.time - _roundStart;

        // 1) contabiliza o evento (contadores do lote)
        string bucket;
        if (foiPego)
        {
            if (elapsed < 10f)      { capt0_10++; bucket = "0_10"; }
            else if (elapsed < 20f) { capt10_20++; bucket = "10_20"; }
            else                    { capt20_30++; bucket = "20_30"; }
        }
        else
        {
            escapes++;
            bucket = "ESCAPE";
            elapsed = roundMaxSeg;
        }

        // 2) log de eventos por round: só até roundsPorLote linhas NO TOTAL
        roundIndexGlobal++;
        if (!_travarEventos)
        {
            if (eventosGravados < roundsPorLote)
            {
                SalvarEventoCsv(roundIndexGlobal, foiPego, elapsed, bucket);
                eventosGravados++;

                if (eventosGravados >= roundsPorLote)
                {
                    _travarEventos = true; // atingiu o limite → não escreve mais
                }
            }
            else
            {
                _travarEventos = true;
            }
        }

        // 3) grava o agregado quando fechar o lote (continua normal)
        roundsFeitosNoLote++;
        if (roundsFeitosNoLote >= roundsPorLote)
        {
            int loteIdx = (roundIndexGlobal + roundsPorLote - 1) / roundsPorLote;
            SalvarLoteNoCsv(loteIdx, capt0_10, capt10_20, capt20_30, escapes, roundsFeitosNoLote);

            roundsFeitosNoLote = 0;
            capt0_10 = capt10_20 = capt20_30 = escapes = 0;
        }

        // ===== respawn em CANTOS OPOSTOS, alternando a diagonal =====
        Bounds b = respawnArea.bounds;

        Corner canto1, canto2;
        if (_usaTL_BR) { canto1 = Corner.TL; canto2 = Corner.BR; }
        else            { canto1 = Corner.TR; canto2 = Corner.BL; }

        bool primeiroEhRunner = _primeiroVaiRunner;

        for (int t = 0; t < tentativas; t++)
        {
            Vector2 p1 = PointOnCorner(b, canto1, margemParede);
            Vector2 p2 = PointOnCorner(b, canto2, margemParede);

            Vector2 pr = primeiroEhRunner ? p1 : p2;
            Vector2 pc = primeiroEhRunner ? p2 : p1;

            if (Physics2D.OverlapCircle(pr, raioRunner, bloqueios)) continue;
            if (Physics2D.OverlapCircle(pc, raioChaser, bloqueios)) continue;
            if ((pr - pc).sqrMagnitude < (minSeparacao * minSeparacao)) continue;

            runnerRb.position = pr;
            chaserRb.position = pc;
            runnerRb.linearVelocity = Vector2.zero;
            chaserRb.linearVelocity = Vector2.zero;

            _nextReset  = Time.time + cooldownSeg;
            _roundStart = Time.time;

            if (alternarDiagonais) _usaTL_BR = !_usaTL_BR;
            if (alternarQuemFicaNoPrimeiroCanto) _primeiroVaiRunner = !_primeiroVaiRunner;
            return;
        }

        // fallback
        Vector2 c = b.center;
        float half = minSeparacao * 0.5f;
        runnerRb.position = c + new Vector2(-half, 0f);
        chaserRb.position = c + new Vector2(+half, 0f);
        runnerRb.linearVelocity = Vector2.zero;
        chaserRb.linearVelocity = Vector2.zero;

        _nextReset  = Time.time + cooldownSeg;
        _roundStart = Time.time;
    }

    void SalvarEventoCsv(int roundIndex, bool capturado, float tempo, string bucket)
    {
        using (var w = new StreamWriter(_csvPathEventos, true))
            w.WriteLine($"{roundIndex},{(capturado ? 1 : 0)},{tempo.ToString("0.###", CultureInfo.InvariantCulture)},{bucket}");
    }

    void SalvarLoteNoCsv(int loteIdx, int c0_10, int c10_20, int c20_30, int esc, int totalRounds)
    {
        using (var w = new StreamWriter(_csvPathAgregado, true))
            w.WriteLine($"{loteIdx},{c0_10},{c10_20},{c20_30},{esc},{totalRounds}");
    }

    enum Corner { TL, TR, BL, BR }

    Vector2 PointOnCorner(Bounds b, Corner c, float inset)
    {
        float xL = b.min.x + inset, xR = b.max.x - inset;
        float yB = b.min.y + inset, yT = b.max.y - inset;
        return c switch
        {
            Corner.TL => new Vector2(xL, yT),
            Corner.TR => new Vector2(xR, yT),
            Corner.BL => new Vector2(xL, yB),
            _         => new Vector2(xR, yB),
        };
    }
}
