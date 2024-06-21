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
                        //AlgoritmoDeKruskal(vertices, listaArestas);
                        break;
                    case 7:
                        //AlgoritmoDeFleury(vertices, listaArestas);
                        break;
                    case 8:
                        //AlgoritmoDeKonigEgervary(vertices, listaArestas);
                        break;
                    case 9:
                        //AlgoritmoGulosoDeColoracao(vertices, listaArestas);
                        break;
                    case 10:
                        //AlgoritmoDeWelshPowell(vertices, listaArestas);
                        break;
                    case 11:
                        //AlgoritmoDeBrelaz(vertices, listaArestas);
                        break;
                    case 12:
                        //AlgoritmoDeKosaraju(vertices, listaArestas);
                        break;
                    case 13:
                        //AlgoritmoDeKahn(vertices, listaArestas);
                        break;
                    case 14:
                        //AlgoritmoDeBellmanFord(vertices, listaArestas);
                        break;
                    case 15:
                        //AlgoritmoDeFordFulkerson(vertices, listaArestas);
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
            // Implementação do Algoritmo de Kruskal
        }

        static void AlgoritmoDeFleury(int vertices, List<(int, int, int)> arestas)
        {
            Console.WriteLine("Executando Algoritmo de Fleury...");
            // Implementação do Algoritmo de Fleury
        }

        static void AlgoritmoDeKonigEgervary(int vertices, List<(int, int, int)> arestas)
        {
            Console.WriteLine("Executando Algoritmo de König-Egerváry...");
            // Implementação do Algoritmo de König-Egerváry
        }

        static void AlgoritmoGulosoDeColoracao(int vertices, List<(int, int, int)> arestas)
        {
            Console.WriteLine("Executando Algoritmo Guloso de Coloração...");
            // Implementação do Algoritmo Guloso de Coloração
        }

        static void AlgoritmoDeWelshPowell(int vertices, List<(int, int, int)> arestas)
        {
            Console.WriteLine("Executando Algoritmo de Welsh-Powell...");
            // Implementação do Algoritmo de Welsh-Powell
        }

        static void AlgoritmoDeBrelaz(int vertices, List<(int, int, int)> arestas)
        {
            Console.WriteLine("Executando Algoritmo de Brélaz...");
            // Implementação do Algoritmo de Brélaz
        }

        static void AlgoritmoDeKosaraju(int vertices, List<(int, int, int)> arestas)
        {
            Console.WriteLine("Executando Algoritmo de Kosaraju...");
            // Implementação do Algoritmo de Kosaraju
        }

        static void AlgoritmoDeKahn(int vertices, List<(int, int, int)> arestas)
        {
            Console.WriteLine("Executando Algoritmo de Kahn...");
            // Implementação do Algoritmo de Kahn
        }

        static void AlgoritmoDeBellmanFord(int vertices, List<(int, int, int)> arestas)
        {
            Console.WriteLine("Executando Algoritmo de Bellman-Ford...");
            // Implementação do Algoritmo de Bellman-Ford
        }

        static void AlgoritmoDeFordFulkerson(int vertices, List<(int, int, int)> arestas)
        {
            Console.WriteLine("Executando Algoritmo de Ford-Fulkerson...");
            // Implementação do Algoritmo de Ford-Fulkerson
        }

    }
}
