using System;
using System.IO;
using System.Collections.Generic;

namespace GrafosApp
{
    public class Program
    {
        public static string arquivo = "./Grafo.txt";

        static void processarGrafo(string caminhoArquivo)
        {

            try
            {
                if (!File.Exists(caminhoArquivo))
                {
                    Console.WriteLine("Erro: O arquivo '{caminhoArquivo}' não foi encontrado.");
                }

                string[] grafo = File.ReadAllLines(caminhoArquivo);

                if (grafo.Length == 0)
                {
                    Console.WriteLine("Erro: O arquivo '{caminhoArquivo}' está vazio.");
                }
                Console.WriteLine("Seu Grafo:");
                for (int i = 0; i < grafo.Length; i++)
                {
                    Console.WriteLine(grafo[i]);
                }

                int opcao = int.Parse(grafo[0]);
                string infos = grafo[1];
                string[] split = infos.Split(' ');
                int vertices = int.Parse(split[0]);
                int arestas = int.Parse(split[1]);

                List<(int, int, int)> listaArestas = new List<(int, int, int)>();
                for (int i = 2; i < grafo.Length; i++)
                {
                    string[] arestaInfo = grafo[i].Split(' ');
                    int origem = int.Parse(arestaInfo[0]);
                    int destino = int.Parse(arestaInfo[1]);
                    int peso = int.Parse(arestaInfo[2]);
                    listaArestas.Add((origem, destino, peso));
                }

                switch (opcao)
                {
                    case 1:
                        BuscaEmProfundidade(vertices, listaArestas, true);
                        break;
                    case 2:
                        BuscaEmLargura(vertices, listaArestas, true);
                        break;
                    case 3:
                        AlgoritmoDeDijkstra(vertices, listaArestas, true);
                        break;
                    case 4:
                        AlgoritmoDePrim(vertices, listaArestas);
                        break;
                    case 5:
                        OrdenacaoTopologica(vertices, listaArestas);
                        break;
                    case 6:
                        AlgoritmoDeKruskal(vertices, listaArestas);
                        break;
                    case 7:
                        AlgoritmoDeFleury(vertices, listaArestas);
                        break;
                    case 8:
                        AlgoritmoDeKonigEgervary(vertices, listaArestas);
                        break;
                    case 9:
                        AlgoritmoGulosoDeColoracao(vertices, listaArestas);
                        break;
                    case 10:
                        AlgoritmoDeWelshPowell(vertices, listaArestas);
                        break;
                    case 11:
                        AlgoritmoDeBrelaz(vertices, listaArestas);
                        break;
                    case 12:
                        AlgoritmoDeKosaraju(vertices, listaArestas);
                        break;
                    case 13:
                        AlgoritmoDeKahn(vertices, listaArestas);
                        break;
                    case 14:
                        int origem = 0;
                        AlgoritmoDeBellmanFord(vertices, listaArestas, origem);
                        break;
                    case 15:
                        AlgoritmoDeFordFulkerson(vertices, listaArestas);
                        break;
                    default:
                        Console.WriteLine("Opção inválida. Tente novamente.");
                        break;
                }
            }
            catch (IOException e)
            {
                Console.WriteLine("Ocorreu um erro ao ler o arquivo: {e.Message}");
            }
            catch (UnauthorizedAccessException)
            {
                Console.WriteLine("Erro: Sem permissão para acessar o arquivo '{caminhoArquivo}'.");
            }
        }

        public static void Main(string[] args)
        {
            Console.Clear();
            Console.WriteLine("--- Tabela de algoritmos ---");
            Console.WriteLine();
            Console.WriteLine("1. Busca em Profundidade (DFS)");
            Console.WriteLine("2. Busca em Largura (BFS)");
            Console.WriteLine("3. Algoritmo de Dijkstra");
            Console.WriteLine("4. Algoritmo de Jarník-Prim");
            Console.WriteLine("5. Ordenação Topológica");
            Console.WriteLine("6. Algoritmo de Kruskal");
            Console.WriteLine("7. Algoritmo de Fleury");
            Console.WriteLine("8. Algoritmo de König-Egerváry");
            Console.WriteLine("9. Algoritmo Guloso de Coloração");
            Console.WriteLine("10. Algoritmo de Welsh-Powell");
            Console.WriteLine("11. Algoritmo de Brélaz");
            Console.WriteLine("12. Algoritmo de Kosaraju");
            Console.WriteLine("13. Algoritmo de Kahn");
            Console.WriteLine("14. Algoritmo de Bellman-Ford");
            Console.WriteLine("15. Algoritmo de Ford-Fulkerson");
            Console.WriteLine();
            Console.WriteLine("Pressione qualquer tecla para executar seu grafo.");
            Console.ReadKey();
            Console.Clear();
            processarGrafo(arquivo);
        }

        static void BuscaEmProfundidade(int vertices, List<(int, int, int)> arestas, bool direcionado)
        {
            Console.WriteLine("Executando Busca em Profundidade (DFS)...");

            // Criação do grafo como uma lista de adjacências
            List<int>[] grafo = new List<int>[vertices];
            for (int i = 0; i < vertices; i++)
            {
                grafo[i] = new List<int>();
            }

            foreach (var aresta in arestas)
            {
                int origem = aresta.Item1;
                int destino = aresta.Item2;
                grafo[origem].Add(destino);
                if (!direcionado)
                {
                    grafo[destino].Add(origem);
                }
            }

            // Array para rastrear os vértices visitados
            bool[] visitado = new bool[vertices];

            // Selecionando o primeiro vértice do grafo como vértice inicial
            int verticeInicial = 0;

            DFS(verticeInicial, grafo, visitado);

            // Para garantir que todos os vértices são visitados (no caso de grafos não conectados)
            for (int i = 0; i < vertices; i++)
            {
                if (!visitado[i])
                {
                    DFS(i, grafo, visitado);
                }
            }
        }

        static void DFS(int vertice, List<int>[] grafo, bool[] visitado)
        {
            // Criação de uma pilha para a DFS iterativa
            Stack<int> pilha = new Stack<int>();
            pilha.Push(vertice);

            while (pilha.Count > 0)
            {
                int v = pilha.Pop();
                if (!visitado[v])
                {
                    visitado[v] = true;
                    Console.WriteLine($"Visitando vértice {v}");
                    foreach (int adj in grafo[v])
                    {
                        if (!visitado[adj])
                        {
                            pilha.Push(adj);
                        }
                    }
                }
            }
        }

        static void BuscaEmLargura(int vertices, List<(int, int, int)> arestas, bool direcionado)
        {
            Console.WriteLine("Executando Busca em Largura (BFS)...");

            // Criação do grafo como uma lista de adjacências
            List<int>[] grafo = new List<int>[vertices];
            for (int i = 0; i < vertices; i++)
            {
                grafo[i] = new List<int>();
            }

            foreach (var aresta in arestas)
            {
                int origem = aresta.Item1;
                int destino = aresta.Item2;
                grafo[origem].Add(destino);
                if (!direcionado)
                {
                    grafo[destino].Add(origem);
                }
            }

            // Array para rastrear os vértices visitados
            bool[] visitado = new bool[vertices];

            // Selecionando o primeiro vértice do grafo como vértice inicial
            int verticeInicial = 0;

            // Fila para a BFS
            Queue<int> fila = new Queue<int>();
            fila.Enqueue(verticeInicial);
            visitado[verticeInicial] = true;

            while (fila.Count > 0)
            {
                int v = fila.Dequeue();
                Console.WriteLine($"Visitando vértice {v}");
                foreach (int adj in grafo[v])
                {
                    if (!visitado[adj])
                    {
                        fila.Enqueue(adj);
                        visitado[adj] = true;
                    }
                }
            }

            // Para garantir que todos os vértices são visitados (no caso de grafos não conectados)
            for (int i = 0; i < vertices; i++)
            {
                if (!visitado[i])
                {
                    fila.Enqueue(i);
                    visitado[i] = true;

                    while (fila.Count > 0)
                    {
                        int v = fila.Dequeue();
                        Console.WriteLine($"Visitando vértice {v}");
                        foreach (int adj in grafo[v])
                        {
                            if (!visitado[adj])
                            {
                                fila.Enqueue(adj);
                                visitado[adj] = true;
                            }
                        }
                    }
                }
            }
        }

        static void AlgoritmoDeDijkstra(int vertices, List<(int, int, int)> arestas, bool direcionado)
        {
            Console.WriteLine("Executando Algoritmo de Dijkstra...");

            // Criação do grafo como uma lista de adjacências
            Dictionary<int, Dictionary<int, int>> grafo = new Dictionary<int, Dictionary<int, int>>();
            for (int i = 0; i < vertices; i++)
            {
                grafo[i] = new Dictionary<int, int>();
            }

            foreach (var aresta in arestas)
            {
                int origem = aresta.Item1;
                int destino = aresta.Item2;
                int peso = aresta.Item3;
                grafo[origem][destino] = peso;
                if (!direcionado)
                {
                    grafo[destino][origem] = peso;
                }
            }

            // Vértice inicial para iniciar o algoritmo de Dijkstra
            int verticeInicial = 0;

            // Distâncias mínimas estimadas a partir do vértice inicial
            Dictionary<int, int> distancia = new Dictionary<int, int>();
            for (int i = 0; i < vertices; i++)
            {
                distancia[i] = int.MaxValue;
            }
            distancia[verticeInicial] = 0;

            // Conjunto de vértices visitados
            HashSet<int> visitados = new HashSet<int>();

            // Algoritmo de Dijkstra
            while (visitados.Count < vertices)
            {
                int verticeAtual = -1;
                int menorDistancia = int.MaxValue;

                // Encontra o vértice não visitado com a menor distância estimada
                foreach (var kvp in distancia)
                {
                    if (!visitados.Contains(kvp.Key) && kvp.Value < menorDistancia)
                    {
                        verticeAtual = kvp.Key;
                        menorDistancia = kvp.Value;
                    }
                }

                // Marca o vértice atual como visitado
                visitados.Add(verticeAtual);

                // Atualiza as distâncias mínimas estimadas para os vértices adjacentes
                foreach (var adjacente in grafo[verticeAtual])
                {
                    int vizinho = adjacente.Key;
                    int peso = adjacente.Value;
                    int novaDistancia = distancia[verticeAtual] + peso;
                    if (novaDistancia < distancia[vizinho])
                    {
                        distancia[vizinho] = novaDistancia;
                    }
                }
            }

            // Imprime as distâncias mínimas estimadas a partir do vértice inicial
            for (int i = 0; i < vertices; i++)
            {
                Console.WriteLine($"Distância mínima de {verticeInicial} para {i}: {distancia[i]}");
            }
        }

        static void AlgoritmoDePrim(int vertices, List<(int, int, int)> arestas)
        {
            Console.WriteLine("Executando Algoritmo de Jarník-Prim...");

            // Criação do grafo como uma lista de adjacências
            List<(int, int)>[] grafo = new List<(int, int)>[vertices];
            for (int i = 0; i < vertices; i++)
            {
                grafo[i] = new List<(int, int)>();
            }

            foreach (var aresta in arestas)
            {
                int origem = aresta.Item1;
                int destino = aresta.Item2;
                int peso = aresta.Item3;
                grafo[origem].Add((destino, peso));
                grafo[destino].Add((origem, peso));
            }

            // Array para rastrear se o vértice foi incluído na MST
            bool[] naMST = new bool[vertices];

            // Array para armazenar o valor mínimo de chave para cada vértice
            int[] chave = new int[vertices];
            for (int i = 0; i < vertices; i++)
            {
                chave[i] = int.MaxValue;
            }

            // Array para armazenar o pai de cada vértice na MST
            int[] pai = new int[vertices];
            pai[0] = -1; // O primeiro vértice é sempre a raiz da MST
            chave[0] = 0;

            // Min-heap ou prioridade para obter o vértice com a menor chave
            SortedSet<(int, int)> prioridade = new SortedSet<(int, int)>();
            prioridade.Add((0, 0)); // (chave, vértice)

            while (prioridade.Count > 0)
            {
                int u = prioridade.Min.Item2;
                prioridade.Remove(prioridade.Min);
                naMST[u] = true;

                foreach (var vizinho in grafo[u])
                {
                    int v = vizinho.Item1;
                    int peso = vizinho.Item2;

                    if (!naMST[v] && peso < chave[v])
                    {
                        prioridade.Remove((chave[v], v));
                        chave[v] = peso;
                        prioridade.Add((chave[v], v));
                        pai[v] = u;
                    }
                }
            }

            // Imprime as arestas da MST
            for (int i = 1; i < vertices; i++)
            {
                Console.WriteLine($"Aresta: {pai[i]} - {i} Peso: {chave[i]}");
            }
        }

        static void OrdenacaoTopologica(int vertices, List<(int, int, int)> arestas)
        {
            Console.WriteLine("Executando Ordenação Topológica...");

            // Criação do grafo como uma lista de adjacências
            List<int>[] grafo = new List<int>[vertices];
            for (int i = 0; i < vertices; i++)
            {
                grafo[i] = new List<int>();
            }

            // Array para armazenar o grau de entrada de cada vértice
            int[] grauDeEntrada = new int[vertices];

            foreach (var aresta in arestas)
            {
                int origem = aresta.Item1;
                int destino = aresta.Item2;
                grafo[origem].Add(destino);
                grauDeEntrada[destino]++;
            }

            // Fila para armazenar os vértices com grau de entrada zero
            Queue<int> fila = new Queue<int>();

            for (int i = 0; i < vertices; i++)
            {
                if (grauDeEntrada[i] == 0)
                {
                    fila.Enqueue(i);
                }
            }

            List<int> ordenacao = new List<int>();

            while (fila.Count > 0)
            {
                int vertice = fila.Dequeue();
                ordenacao.Add(vertice);

                foreach (var adjacente in grafo[vertice])
                {
                    grauDeEntrada[adjacente]--;
                    if (grauDeEntrada[adjacente] == 0)
                    {
                        fila.Enqueue(adjacente);
                    }
                }
            }

            if (ordenacao.Count != vertices)
            {
                Console.WriteLine("O grafo contém um ciclo e, portanto, não pode ser ordenado topologicamente.");
            }
            else
            {
                Console.WriteLine("Ordenação Topológica:");
                foreach (var vertice in ordenacao)
                {
                    Console.Write($"{vertice} ");
                }
                Console.WriteLine();
            }
        }

        static void AlgoritmoDeKruskal(int vertices, List<(int, int, int)> arestas)
        {
            Console.WriteLine("Executando Algoritmo de Kruskal...");

            // Ordena as arestas pelo peso
            var arestasOrdenadas = arestas.OrderBy(aresta => aresta.Item3).ToList();

            // Inicializa a estrutura de conjuntos disjuntos
            var dsu = new DisjointSetUnion(vertices);

            List<(int, int, int)> mst = new List<(int, int, int)>();

            foreach (var aresta in arestasOrdenadas)
            {
                int origem = aresta.Item1;
                int destino = aresta.Item2;
                int peso = aresta.Item3;

                if (dsu.Find(origem) != dsu.Find(destino))
                {
                    dsu.Union(origem, destino);
                    mst.Add((origem, destino, peso));
                }
            }

            // Imprime a MST
            Console.WriteLine("Árvore Geradora Mínima (MST):");
            foreach (var aresta in mst)
            {
                Console.WriteLine($"{aresta.Item1} - {aresta.Item2} (peso: {aresta.Item3})");
            }
        }

        static void AlgoritmoDeFleury(int vertices, List<(int, int, int)> arestas)
        {
            Console.WriteLine("Executando Algoritmo de Fleury...");

            // Criação do grafo como uma lista de adjacências
            List<int>[] grafo = new List<int>[vertices];
            for (int i = 0; i < vertices; i++)
            {
                grafo[i] = new List<int>();
            }

            foreach (var aresta in arestas)
            {
                int origem = aresta.Item1;
                int destino = aresta.Item2;

                if (origem < vertices && destino < vertices) // Verificação de limite
                {
                    grafo[origem].Add(destino);
                    grafo[destino].Add(origem);
                }
                else
                {
                    Console.WriteLine("Aresta fora dos limites do grafo: ({0}, {1})", origem, destino);
                }
            }

            List<int> circuito = new List<int>();
            Fleury(0, grafo, circuito);

            Console.WriteLine("Circuito Euleriano:");
            foreach (var vertice in circuito)
            {
                Console.Write($"{vertice} ");
            }
            Console.WriteLine();
        }

        static void Fleury(int vertice, List<int>[] grafo, List<int> circuito)
        {
            for (int i = 0; i < grafo[vertice].Count; i++)
            {
                int adj = grafo[vertice][i];

                // Verifica se a aresta é uma ponte
                if (adj != -1 && !IsBridge(vertice, adj, grafo))
                {
                    Console.WriteLine($"Removendo aresta {vertice} - {adj}");
                    RemoveAresta(vertice, adj, grafo);
                    Fleury(adj, grafo, circuito);
                }
            }
            circuito.Add(vertice);
        }

        static void RemoveAresta(int u, int v, List<int>[] grafo)
        {
            grafo[u].Remove(v);
            grafo[v].Remove(u);
        }

        static bool IsBridge(int u, int v, List<int>[] grafo)
        {
            int count1 = DFSCount(u, grafo);

            RemoveAresta(u, v, grafo);

            int count2 = DFSCount(u, grafo);

            AddAresta(u, v, grafo);

            return count1 > count2;
        }

        static void AddAresta(int u, int v, List<int>[] grafo)
        {
            grafo[u].Add(v);
            grafo[v].Add(u);
        }

        static int DFSCount(int v, List<int>[] grafo)
        {
            bool[] visitado = new bool[grafo.Length];
            Stack<int> pilha = new Stack<int>();
            pilha.Push(v);
            visitado[v] = true;
            int count = 1;

            while (pilha.Count > 0)
            {
                int vertice = pilha.Pop();

                foreach (int adj in grafo[vertice])
                {
                    if (!visitado[adj])
                    {
                        pilha.Push(adj);
                        visitado[adj] = true;
                        count++;
                    }
                }
            }

            return count;
        }

        static void AlgoritmoDeKonigEgervary(int vertices, List<(int, int, int)> arestas)
        {
            Console.WriteLine("Executando Algoritmo de König-Egerváry...");

            // Separando as duas partições do grafo bipartido
            int[] U = new int[vertices / 2];
            int[] V = new int[vertices - vertices / 2];
            for (int i = 0; i < U.Length; i++)
                U[i] = i;
            for (int i = 0; i < V.Length; i++)
                V[i] = i + U.Length;

            // Criação do grafo bipartido
            Dictionary<int, List<int>> grafo = new Dictionary<int, List<int>>();
            foreach (var u in U)
                grafo[u] = new List<int>();
            foreach (var v in V)
                grafo[v] = new List<int>(); // Adiciona os vértices da partição V também

            foreach (var aresta in arestas)
            {
                int origem = aresta.Item1;
                int destino = aresta.Item2;
                grafo[origem].Add(destino);
            }

            // Inicialização do emparelhamento
            Dictionary<int, int> parU = new Dictionary<int, int>();
            Dictionary<int, int> parV = new Dictionary<int, int>();
            foreach (var u in U)
                parU[u] = -1;
            foreach (var v in V)
                parV[v] = -1;

            // Função de BFS para encontrar caminhos aumentantes
            bool BFS(Dictionary<int, int> parU, Dictionary<int, int> parV, Dictionary<int, List<int>> grafo, Dictionary<int, int> dist)
            {
                Queue<int> fila = new Queue<int>();
                foreach (var u in U)
                {
                    if (parU[u] == -1)
                    {
                        dist[u] = 0;
                        fila.Enqueue(u);
                    }
                    else
                    {
                        dist[u] = int.MaxValue; // Inicializa a distância para todos os vértices
                    }
                }

                // Inicializa a distância para vértices em V
                foreach (var v in V)
                {
                    dist[v] = int.MaxValue;
                }

                dist[-1] = int.MaxValue;
                while (fila.Count > 0)
                {
                    int u = fila.Dequeue();
                    if (dist[u] < dist[-1])
                    {
                        foreach (var v in grafo[u])
                        {
                            if (dist[parV[v]] == int.MaxValue)
                            {
                                dist[parV[v]] = dist[u] + 1;
                                fila.Enqueue(parV[v]);
                            }
                        }
                    }
                }
                return dist[-1] != int.MaxValue;
            }


            // Função de DFS para encontrar e aumentar emparelhamentos
            bool DFS(int u, Dictionary<int, int> parU, Dictionary<int, int> parV, Dictionary<int, List<int>> grafo, Dictionary<int, int> dist)
            {
                if (u != -1)
                {
                    foreach (var v in grafo[u])
                    {
                        if (dist[parV[v]] == dist[u] + 1)
                        {
                            if (DFS(parV[v], parU, parV, grafo, dist))
                            {
                                parV[v] = u;
                                parU[u] = v;
                                return true;
                            }
                        }
                    }
                    dist[u] = int.MaxValue;
                    return false;
                }
                return true;
            }

            // Algoritmo de Hopcroft-Karp para encontrar o emparelhamento máximo
            int hopcroftKarp(Dictionary<int, int> parU, Dictionary<int, int> parV, Dictionary<int, List<int>> grafo)
            {
                int emparelhamento = 0;
                Dictionary<int, int> dist = new Dictionary<int, int>();
                while (BFS(parU, parV, grafo, dist))
                {
                    foreach (var u in U)
                    {
                        if (parU[u] == -1)
                        {
                            if (DFS(u, parU, parV, grafo, dist))
                            {
                                emparelhamento++;
                            }
                        }
                    }
                }
                return emparelhamento;
            }

            int maxEmparelhamento = hopcroftKarp(parU, parV, grafo);
            Console.WriteLine($"O tamanho do emparelhamento máximo é: {maxEmparelhamento}");

            Console.WriteLine("Emparelhamento máximo:");
            foreach (var u in U)
            {
                if (parU[u] != -1)
                {
                    Console.WriteLine($"{u} - {parU[u]}");
                }
            }
        }

        public static void AlgoritmoGulosoDeColoracao(int vertices, List<(int, int, int)> listaArestas)
        {
            Dictionary<int, int> cores = new Dictionary<int, int>();

            for (int v = 0; v < vertices; v++)
            {
                // Lista de cores usadas pelos vizinhos
                List<int> coresVizinhos = new List<int>();

                // Percorre todas as arestas
                foreach (var aresta in listaArestas)
                {
                    int origem = aresta.Item1;
                    int destino = aresta.Item2;

                    // Verifica se o vértice atual é a origem ou o destino da aresta
                    if (origem == v || destino == v)
                    {
                        // Se o vértice oposto já tiver uma cor atribuída, adiciona essa cor à lista de cores dos vizinhos
                        if (cores.ContainsKey(origem) && origem != v)
                        {
                            coresVizinhos.Add(cores[origem]);
                        }
                        if (cores.ContainsKey(destino) && destino != v)
                        {
                            coresVizinhos.Add(cores[destino]);
                        }
                    }
                }

                // Escolhe a menor cor disponível para o vértice
                int cor = 0;
                while (coresVizinhos.Contains(cor))
                {
                    cor++;
                }

                // Atribui a cor ao vértice
                cores[v] = cor;
            }

            // Exibe a coloração
            Console.WriteLine("\nColoração dos vértices:");
            foreach (var vertice in cores)
            {
                Console.WriteLine($"Vértice {vertice.Key} --> Cor {vertice.Value}");
            }
        }

        static void AlgoritmoDeWelshPowell(int vertices, List<(int, int, int)> arestas)
        {
            Console.WriteLine("Executando Algoritmo de Welsh-Powell...");

            // Criação do grafo como uma lista de adjacências
            List<int>[] grafo = new List<int>[vertices];
            for (int i = 0; i < vertices; i++)
            {
                grafo[i] = new List<int>();
            }

            foreach (var aresta in arestas)
            {
                int origem = aresta.Item1;
                int destino = aresta.Item2;
                grafo[origem].Add(destino);
                grafo[destino].Add(origem); // Grafo não direcionado
            }

            // Ordena os vértices em ordem decrescente de grau
            int[] graus = new int[vertices];
            for (int i = 0; i < vertices; i++)
            {
                graus[i] = grafo[i].Count;
            }

            int[] verticesOrdenados = Enumerable.Range(0, vertices).OrderByDescending(v => graus[v]).ToArray();

            // Array para armazenar as cores dos vértices
            int[] cores = new int[vertices];
            for (int i = 0; i < vertices; i++)
            {
                cores[i] = -1; // -1 significa que o vértice ainda não foi colorido
            }

            // Atribui a menor cor disponível a cada vértice
            for (int i = 0; i < vertices; i++)
            {
                int v = verticesOrdenados[i];

                // Array para marcar as cores disponíveis
                bool[] coresDisponiveis = new bool[vertices];
                for (int j = 0; j < vertices; j++)
                {
                    coresDisponiveis[j] = true;
                }

                // Marca as cores já usadas pelos vizinhos
                foreach (int vizinho in grafo[v])
                {
                    if (cores[vizinho] != -1)
                    {
                        coresDisponiveis[cores[vizinho]] = false;
                    }
                }

                // Encontra a menor cor disponível
                int cor;
                for (cor = 0; cor < vertices; cor++)
                {
                    if (coresDisponiveis[cor])
                    {
                        break;
                    }
                }

                cores[v] = cor;
            }

            // Imprime as cores dos vértices
            Console.WriteLine("Coloração dos vértices:");
            for (int i = 0; i < vertices; i++)
            {
                Console.WriteLine($"Vértice {i}: Cor {cores[i]}");
            }
        }

        static void AlgoritmoDeBrelaz(int vertices, List<(int, int, int)> arestas)
        {
            Console.WriteLine("Executando Algoritmo de Brélaz...");

            // Criação do grafo como uma lista de adjacências
            List<int>[] grafo = new List<int>[vertices];
            for (int i = 0; i < vertices; i++)
            {
                grafo[i] = new List<int>();
            }

            foreach (var aresta in arestas)
            {
                int origem = aresta.Item1;
                int destino = aresta.Item2;
                grafo[origem].Add(destino);
                grafo[destino].Add(origem); // Grafo não direcionado
            }

            // Array para armazenar as cores dos vértices
            int[] cores = new int[vertices];
            for (int i = 0; i < vertices; i++)
            {
                cores[i] = -1; // -1 significa que o vértice ainda não foi colorido
            }

            // Array para armazenar a saturação dos vértices
            int[] saturacao = new int[vertices];
            for (int i = 0; i < vertices; i++)
            {
                saturacao[i] = 0;
            }

            // Ordena os vértices por grau inicial
            int[] graus = new int[vertices];
            for (int i = 0; i < vertices; i++)
            {
                graus[i] = grafo[i].Count;
            }

            int[] verticesOrdenados = Enumerable.Range(0, vertices).OrderByDescending(v => graus[v]).ToArray();

            // Atribui a menor cor disponível ao vértice com maior saturação
            for (int i = 0; i < vertices; i++)
            {
                // Seleciona o vértice com maior saturação (em caso de empate, escolhe o de maior grau)
                int maxSaturacao = -1;
                int v = -1;
                foreach (int u in verticesOrdenados)
                {
                    if (cores[u] == -1)
                    {
                        if (saturacao[u] > maxSaturacao || (saturacao[u] == maxSaturacao && graus[u] > graus[v]))
                        {
                            maxSaturacao = saturacao[u];
                            v = u;
                        }
                    }
                }

                // Array para marcar as cores disponíveis
                bool[] coresDisponiveis = new bool[vertices];
                for (int j = 0; j < vertices; j++)
                {
                    coresDisponiveis[j] = true;
                }

                // Marca as cores já usadas pelos vizinhos
                foreach (int vizinho in grafo[v])
                {
                    if (cores[vizinho] != -1)
                    {
                        coresDisponiveis[cores[vizinho]] = false;
                    }
                }

                // Encontra a menor cor disponível
                int cor;
                for (cor = 0; cor < vertices; cor++)
                {
                    if (coresDisponiveis[cor])
                    {
                        break;
                    }
                }

                cores[v] = cor;

                // Atualiza a saturação dos vizinhos
                foreach (int vizinho in grafo[v])
                {
                    if (cores[vizinho] == -1)
                    {
                        int coresDiferentes = grafo[vizinho].Select(x => cores[x]).Distinct().Count(c => c != -1);
                        saturacao[vizinho] = coresDiferentes;
                    }
                }
            }

            // Imprime as cores dos vértices
            Console.WriteLine("Coloração dos vértices:");
            for (int i = 0; i < vertices; i++)
            {
                Console.WriteLine($"Vértice {i}: Cor {cores[i]}");
            }
        }

        static void AlgoritmoDeKosaraju(int vertices, List<(int, int, int)> arestas)
        {
            Console.WriteLine("Executando Algoritmo de Kosaraju...");

            // Criação do grafo como uma lista de adjacências
            List<int>[] grafo = new List<int>[vertices];
            for (int i = 0; i < vertices; i++)
            {
                grafo[i] = new List<int>();
            }

            foreach (var aresta in arestas)
            {
                int origem = aresta.Item1;
                int destino = aresta.Item2;
                grafo[origem].Add(destino);
            }

            // Passo 1: Realizar uma DFS e armazenar a ordem dos vértices conforme são finalizados
            Stack<int> ordemDeTermino = new Stack<int>();
            bool[] visitado = new bool[vertices];

            for (int i = 0; i < vertices; i++)
            {
                if (!visitado[i])
                {
                    DFSComOrdemDeTermino(i, grafo, visitado, ordemDeTermino);
                }
            }

            // Passo 2: Inverter o grafo
            List<int>[] grafoInvertido = new List<int>[vertices];
            for (int i = 0; i < vertices; i++)
            {
                grafoInvertido[i] = new List<int>();
            }

            foreach (var aresta in arestas)
            {
                int origem = aresta.Item1;
                int destino = aresta.Item2;
                grafoInvertido[destino].Add(origem);
            }

            // Passo 3: Realizar uma DFS no grafo invertido na ordem dos vértices armazenados
            for (int i = 0; i < vertices; i++)
            {
                visitado[i] = false;
            }

            int componenteId = 0;
            List<List<int>> componentesFortementeConectadas = new List<List<int>>();

            while (ordemDeTermino.Count > 0)
            {
                int v = ordemDeTermino.Pop();
                if (!visitado[v])
                {
                    List<int> componente = new List<int>();
                    DFSComponentes(v, grafoInvertido, visitado, componente);
                    componentesFortementeConectadas.Add(componente);
                    componenteId++;
                }
            }

            // Imprimir as componentes fortemente conectadas
            Console.WriteLine("Componentes Fortemente Conectadas:");
            for (int i = 0; i < componentesFortementeConectadas.Count; i++)
            {
                Console.WriteLine($"Componente {i}: {string.Join(", ", componentesFortementeConectadas[i])}");
            }
        }

        static void DFSComOrdemDeTermino(int vertice, List<int>[] grafo, bool[] visitado, Stack<int> ordemDeTermino)
        {
            // Criação de uma pilha para a DFS iterativa
            Stack<int> pilha = new Stack<int>();
            pilha.Push(vertice);

            while (pilha.Count > 0)
            {
                int v = pilha.Pop();
                if (!visitado[v])
                {
                    visitado[v] = true;
                    foreach (int adj in grafo[v])
                    {
                        if (!visitado[adj])
                        {
                            pilha.Push(adj);
                        }
                    }
                    ordemDeTermino.Push(v);
                }
            }
        }

        static void DFSComponentes(int vertice, List<int>[] grafo, bool[] visitado, List<int> componente)
        {
            // Criação de uma pilha para a DFS iterativa
            Stack<int> pilha = new Stack<int>();
            pilha.Push(vertice);

            while (pilha.Count > 0)
            {
                int v = pilha.Pop();
                if (!visitado[v])
                {
                    visitado[v] = true;
                    componente.Add(v);
                    foreach (int adj in grafo[v])
                    {
                        if (!visitado[adj])
                        {
                            pilha.Push(adj);
                        }
                    }
                }
            }
        }

        static void AlgoritmoDeKahn(int vertices, List<(int, int, int)> arestas)
        {
            Console.WriteLine("Executando Algoritmo de Kahn...");

            // Criação do grafo como uma lista de adjacências
            List<int>[] grafo = new List<int>[vertices];
            for (int i = 0; i < vertices; i++)
            {
                grafo[i] = new List<int>();
            }

            // Array para contar os graus de entrada dos vértices
            int[] grauDeEntrada = new int[vertices];

            // Preenchimento do grafo e cálculo dos graus de entrada
            foreach (var aresta in arestas)
            {
                int origem = aresta.Item1;
                int destino = aresta.Item2;
                grafo[origem].Add(destino);
                grauDeEntrada[destino]++;
            }

            // Fila para armazenar os vértices com grau de entrada zero
            Queue<int> fila = new Queue<int>();
            for (int i = 0; i < vertices; i++)
            {
                if (grauDeEntrada[i] == 0)
                {
                    fila.Enqueue(i);
                }
            }

            // Lista para armazenar a ordenação topológica
            List<int> ordenacaoTopologica = new List<int>();

            // Processamento dos vértices
            while (fila.Count > 0)
            {
                int v = fila.Dequeue();
                ordenacaoTopologica.Add(v);

                foreach (int adj in grafo[v])
                {
                    grauDeEntrada[adj]--;
                    if (grauDeEntrada[adj] == 0)
                    {
                        fila.Enqueue(adj);
                    }
                }
            }

            // Verificação de ciclo no grafo
            if (ordenacaoTopologica.Count != vertices)
            {
                Console.WriteLine("O grafo possui um ciclo e não pode ser ordenado topologicamente.");
            }
            else
            {
                // Imprimir a ordenação topológica
                Console.WriteLine("Ordenação Topológica:");
                Console.WriteLine(string.Join(", ", ordenacaoTopologica));
            }
        }

        static void AlgoritmoDeBellmanFord(int vertices, List<(int, int, int)> arestas, int origem)
        {
            Console.WriteLine("Executando Algoritmo de Bellman-Ford...");

            // Inicialização das distâncias
            int[] distancias = new int[vertices];
            for (int i = 0; i < vertices; i++)
            {
                distancias[i] = int.MaxValue;
            }
            distancias[origem] = 0;

            // Relaxamento das arestas
            for (int i = 0; i < vertices - 1; i++)
            {
                foreach (var aresta in arestas)
                {
                    int u = aresta.Item1;
                    int v = aresta.Item2;
                    int peso = aresta.Item3;
                    if (distancias[u] != int.MaxValue && distancias[u] + peso < distancias[v])
                    {
                        distancias[v] = distancias[u] + peso;
                    }
                }
            }

            // Verificação de ciclos negativos
            foreach (var aresta in arestas)
            {
                int u = aresta.Item1;
                int v = aresta.Item2;
                int peso = aresta.Item3;
                if (distancias[u] != int.MaxValue && distancias[u] + peso < distancias[v])
                {
                    Console.WriteLine("O grafo contém um ciclo de peso negativo.");
                    return;
                }
            }

            // Impressão das distâncias
            Console.WriteLine("Distâncias mínimas a partir do vértice de origem:");
            for (int i = 0; i < vertices; i++)
            {
                if (distancias[i] == int.MaxValue)
                {
                    Console.WriteLine($"Vértice {i}: Infinito");
                }
                else
                {
                    Console.WriteLine($"Vértice {i}: {distancias[i]}");
                }
            }
        }

         static void AlgoritmoDeFordFulkerson(int vertices, List<(int, int, int)> arestas)
        {
            Console.WriteLine("Executando Algoritmo de Ford-Fulkerson...");

            // Criação do grafo residual como uma matriz de adjacências
            int[,] grafoResidual = new int[vertices, vertices];

            foreach (var aresta in arestas)
            {
                int origem = aresta.Item1;
                int destino = aresta.Item2;
                int capacidade = aresta.Item3;
                grafoResidual[origem, destino] = capacidade;
            }

            int source = 0; // Vértice de origem
            int sink = vertices - 1; // Vértice de destino

            // Algoritmo de Ford-Fulkerson usando DFS para encontrar caminhos aumentantes
            int fluxoMaximo = 0;
            int[] pai = new int[vertices];
            while (DFS(source, sink, pai, grafoResidual, vertices))
            {
                int fluxoCaminho = int.MaxValue;
                for (int v = sink; v != source; v = pai[v])
                {
                    int u = pai[v];
                    fluxoCaminho = Math.Min(fluxoCaminho, grafoResidual[u, v]);
                }

                for (int v = sink; v != source; v = pai[v])
                {
                    int u = pai[v];
                    grafoResidual[u, v] -= fluxoCaminho;
                    grafoResidual[v, u] += fluxoCaminho;
                }

                fluxoMaximo += fluxoCaminho;
            }

            Console.WriteLine($"Fluxo máximo: {fluxoMaximo}");
        }

        static bool DFS(int source, int sink, int[] pai, int[,] grafoResidual, int vertices)
        {
            bool[] visitado = new bool[vertices];
            Stack<int> pilha = new Stack<int>();
            pilha.Push(source);
            visitado[source] = true;
            pai[source] = -1;

            while (pilha.Count > 0)
            {
                int u = pilha.Pop();

                for (int v = 0; v < vertices; v++)
                {
                    if (!visitado[v] && grafoResidual[u, v] > 0)
                    {
                        pilha.Push(v);
                        pai[v] = u;
                        visitado[v] = true;
                    }
                }
            }

            return visitado[sink];
        }

    }

    // Classe para representar a estrutura de Conjuntos Disjuntos
    class DisjointSetUnion
    {
        private int[] pai;
        private int[] rank;

        public DisjointSetUnion(int n)
        {
            pai = new int[n];
            rank = new int[n];
            for (int i = 0; i < n; i++)
            {
                pai[i] = i;
                rank[i] = 0;
            }
        }

        public int Find(int u)
        {
            if (pai[u] != u)
            {
                pai[u] = Find(pai[u]);
            }
            return pai[u];
        }

        public void Union(int u, int v)
        {
            int raizU = Find(u);
            int raizV = Find(v);

            if (raizU != raizV)
            {
                if (rank[raizU] < rank[raizV])
                {
                    pai[raizU] = raizV;
                }
                else if (rank[raizU] > rank[raizV])
                {
                    pai[raizV] = raizU;
                }
                else
                {
                    pai[raizV] = raizU;
                    rank[raizU]++;
                }
            }
        }
    }
}
