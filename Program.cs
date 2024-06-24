using System;
using System.IO;
using System.Collections.Generic;

namespace GrafosApp
{
    public class Program
    {

        static void processarGrafo()
        {
            int opcao = 0;
            int cont = 0;
            int vertices = 0;
            int arestas = 0;
            int verticeInicial = 0;
            List<string> lista = new List<string>();
            try
            {
                string entrada = Console.ReadLine();
                while(entrada != null){
                    if(cont == 0){
                        opcao = int.Parse(entrada);
                    }
                    if(cont == 1){
                        string[] split = entrada.Split(' ');
                        vertices = int.Parse(split[0]);
                        arestas = int.Parse(split[1]);
                    }
                    if(cont == 2){
                        string[] split = entrada.Split(' ');
                        verticeInicial = int.Parse(split[0]);
                        lista.Add(entrada);
                    }
                    if(cont >= 3){
                        lista.Add(entrada);
                    }
                    cont++;
                    entrada = Console.ReadLine();
                }

                string[] grafo = lista.ToArray();
                Console.WriteLine("Seu Grafo:");
                for (int i = 0; i < grafo.Length; i++)
                {
                    Console.WriteLine(grafo[i]);
                }

                List<(int, int, int)> listaArestas = new List<(int, int, int)>();
                for (int i = 0; i < grafo.Length; i++)
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
                        BuscaEmProfundidade(vertices, listaArestas, verticeInicial);
                        break;
                    case 2:
                        BuscaEmLargura(vertices, listaArestas, verticeInicial);
                        break;
                    case 3:
                        AlgoritmoDeDijkstra(vertices, listaArestas, verticeInicial);
                        break;
                    case 4:
                        AlgoritmoDePrim(vertices, listaArestas, verticeInicial);
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
                        AlgoritmoDeBellmanFord(vertices, listaArestas, verticeInicial);
                        break;
                    case 15:
                        AlgoritmoDeFordFulkerson(vertices, listaArestas, verticeInicial);
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
            processarGrafo();
        }

        static void BuscaEmProfundidade(int vertices, List<(int, int, int)> arestas, int verticeInicial)
        {
            Console.WriteLine("Executando Busca em profundidade...");
            // Criando uma lista de adjacência
            List<int>[] adjList = new List<int>[vertices];
            for (int i = 0; i < vertices; i++)
            {
                adjList[i] = new List<int>();
            }

            // Preenchendo a lista de adjacência com as arestas direcionadas
            foreach (var aresta in arestas)
            {
                adjList[aresta.Item1].Add(aresta.Item2);
            }

            // Criando um array para marcar os vértices visitados
            bool[] visitados = new bool[vertices];

            // Lista para armazenar a ordem de visita
            List<int> ordemDeVisita = new List<int>();

            // Função auxiliar para realizar a DFS
            void DFS(int vertice)
            {
                visitados[vertice] = true;
                ordemDeVisita.Add(vertice);

                foreach (var vizinho in adjList[vertice])
                {
                    if (!visitados[vizinho])
                    {
                        DFS(vizinho);
                    }
                }
            }

            // Chamando a DFS a partir do vértice inicial
            DFS(verticeInicial);

            // Imprimindo a ordem de visita
            Console.WriteLine("Ordem de Visita:");
            foreach (var vertice in ordemDeVisita)
            {
                Console.Write(vertice + " ");
            }
        }

        static void BuscaEmLargura(int vertices, List<(int, int, int)> arestas, int verticeInicial)
        {
            Console.WriteLine("Executando Busca em largura...");
            // Criando uma lista de adjacência
            List<int>[] adjList = new List<int>[vertices];
            for (int i = 0; i < vertices; i++)
            {
                adjList[i] = new List<int>();
            }

            // Preenchendo a lista de adjacência com as arestas direcionadas
            foreach (var aresta in arestas)
            {
                adjList[aresta.Item1].Add(aresta.Item2);
            }

            // Criando um array para marcar os vértices visitados
            bool[] visitados = new bool[vertices];

            // Lista para armazenar a ordem de visita
            List<int> ordemDeVisita = new List<int>();

            // Fila para gerenciar a BFS
            Queue<int> fila = new Queue<int>();

            // Iniciando a BFS a partir do vértice inicial
            visitados[verticeInicial] = true;
            fila.Enqueue(verticeInicial);

            while (fila.Count > 0)
            {
                int vertice = fila.Dequeue();
                ordemDeVisita.Add(vertice);

                foreach (var vizinho in adjList[vertice])
                {
                    if (!visitados[vizinho])
                    {
                        visitados[vizinho] = true;
                        fila.Enqueue(vizinho);
                    }
                }
            }

            // Imprimindo a ordem de visita
            Console.WriteLine("Ordem de Visita:");
            foreach (var vertice in ordemDeVisita)
            {
                Console.Write(vertice + " ");
            }
        }

        static void AlgoritmoDeDijkstra(int vertices, List<(int, int, int)> arestas, int verticeInicial)
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
            }

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

        static void AlgoritmoDePrim(int vertices, List<(int, int, int)> arestas, int verticeInicial)
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
            pai[verticeInicial] = -1; // O primeiro vértice é sempre a raiz da MST
            chave[verticeInicial] = 0;

            // Min-heap ou prioridade para obter o vértice com a menor chave
            SortedSet<(int, int)> prioridade = new SortedSet<(int, int)>();
            prioridade.Add((0, verticeInicial)); // (chave, vértice)

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

            // Preenchimento do grafo
            foreach (var aresta in arestas)
            {
                int origem = aresta.Item1;
                int destino = aresta.Item2;
                grafo[origem].Add(destino);
            }

            // Marca os vértices visitados
            bool[] visitado = new bool[vertices];
            Stack<int> pilha = new Stack<int>();

            // Função auxiliar para DFS
            void DFS(int v)
            {
                visitado[v] = true;

                foreach (int adj in grafo[v])
                {
                    if (!visitado[adj])
                    {
                        DFS(adj);
                    }
                }

                pilha.Push(v);
            }

            // Executando DFS para todos os vértices
            for (int i = 0; i < vertices; i++)
            {
                if (!visitado[i])
                {
                    DFS(i);
                }
            }

            // Verificando se a ordenação topológica é possível
            if (pilha.Count != vertices)
            {
                Console.WriteLine("O grafo contém um ciclo e, portanto, não pode ser ordenado topologicamente.");
            }
            else
            {
                Console.WriteLine("Ordenação Topológica:");
                while (pilha.Count > 0)
                {
                    Console.Write($"{pilha.Pop()} ");
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

            // Criação do grafo não direcionado
            Dictionary<int, List<int>> grafo = new Dictionary<int, List<int>>();
            for (int i = 0; i < vertices; i++)
            {
                grafo[i] = new List<int>();
            }

            // Preenchendo o grafo com as arestas
            foreach (var aresta in arestas)
            {
                int u = aresta.Item1;
                int v = aresta.Item2;
                grafo[u].Add(v);
                grafo[v].Add(u); // O grafo é não direcionado, então adicionamos as arestas nos dois sentidos
            }

            // Arrays para o emparelhamento e a cobertura mínima
            int[] emparelhamento = new int[vertices];
            int[] coberturaMinima = new int[vertices];
            Array.Fill(emparelhamento, -1); // Inicializa com -1 indicando que nenhum vértice está emparelhado

            // Função para encontrar o caminho aumentante usando DFS
            bool EncontrarCaminhoAumentante(int u, bool[] visitado)
            {
                foreach (int v in grafo[u])
                {
                    // Se o vértice v não foi visitado ainda
                    if (!visitado[v])
                    {
                        visitado[v] = true;

                        // Se v não está emparelhado ou se encontramos um caminho aumentante para o parceiro de v
                        if (emparelhamento[v] == -1 || EncontrarCaminhoAumentante(emparelhamento[v], visitado))
                        {
                            emparelhamento[v] = u;
                            return true;
                        }
                    }
                }
                return false;
            }

            // Algoritmo principal de König-Egerváry
            int tamanhoEmparelhamentoMaximo = 0;
            for (int u = 0; u < vertices; u++)
            {
                // Array para marcar os vértices visitados em cada DFS
                bool[] visitado = new bool[vertices];
                if (EncontrarCaminhoAumentante(u, visitado))
                {
                    tamanhoEmparelhamentoMaximo++;
                }
            }

            Console.WriteLine($"Tamanho do emparelhamento máximo é: {tamanhoEmparelhamentoMaximo}");

            Console.WriteLine("Emparelhamento máximo:");
            for (int v = 0; v < vertices; v++)
            {
                if (emparelhamento[v] != -1)
                {
                    Console.WriteLine($"{emparelhamento[v]} - {v}");
                }
            }
        }

        public static void AlgoritmoGulosoDeColoracao(int vertices, List<(int, int, int)> listaArestas)
        {
            Console.WriteLine("Executando Algoritmo Guloso de Coloração...");
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

        static void AlgoritmoDeBellmanFord(int vertices, List<(int, int, int)> arestas, int verticeInicial)
        {
            Console.WriteLine("Executando Algoritmo de Bellman-Ford...");

            // Inicialização das distâncias
            int[] distancias = new int[vertices];
            for (int i = 0; i < vertices; i++)
            {
                distancias[i] = int.MaxValue;
            }
            distancias[verticeInicial] = 0;

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

         static void AlgoritmoDeFordFulkerson(int vertices, List<(int, int, int)> arestas, int verticeInicial)
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

            int verticeDestino = vertices - 1; // Vértice de destino

            // Algoritmo de Ford-Fulkerson usando DFS para encontrar caminhos aumentantes
            int fluxoMaximo = 0;
            int[] pai = new int[vertices];
            while (DFS(verticeInicial, verticeDestino, pai, grafoResidual, vertices))
            {
                int fluxoCaminho = int.MaxValue;
                for (int v = verticeDestino; v != verticeInicial; v = pai[v])
                {
                    int u = pai[v];
                    fluxoCaminho = Math.Min(fluxoCaminho, grafoResidual[u, v]);
                }

                for (int v = verticeDestino; v != verticeInicial; v = pai[v])
                {
                    int u = pai[v];
                    grafoResidual[u, v] -= fluxoCaminho;
                    grafoResidual[v, u] += fluxoCaminho;
                }

                fluxoMaximo += fluxoCaminho;
            }

            Console.WriteLine($"Fluxo máximo: {fluxoMaximo}");
        }

        static bool DFS(int verticeInicial, int verticeDestino, int[] pai, int[,] grafoResidual, int vertices)
        {
            bool[] visitado = new bool[vertices];
            Stack<int> pilha = new Stack<int>();
            pilha.Push(verticeInicial);
            visitado[verticeInicial] = true;
            pai[verticeInicial] = -1;

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

            return visitado[verticeDestino];
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
