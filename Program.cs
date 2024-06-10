﻿using System;
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
                        //BuscaEmLargura(vertices, listaArestas);
                        break;
                    case 3:
                        //AlgoritmoDeDijkstra(vertices, listaArestas);
                        break;
                    case 4:
                        //AlgoritmoDePrim(vertices, listaArestas);
                        break;
                    case 5:
                        //OrdenacaoTopologica(vertices, listaArestas);
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

        static void BuscaEmLargura(int vertices, int arestas)
        {
            Console.WriteLine("Executando Busca em Largura (BFS)...");
            // Implementação do Algoritmo de Busca em Largura
        }

        static void AlgoritmoDeDijkstra(int vertices, int arestas)
        {
            Console.WriteLine("Executando Algoritmo de Dijkstra...");
            // Implementação do Algoritmo de Dijkstra
        }

        static void AlgoritmoDePrim(int vertices, int arestas)
        {
            Console.WriteLine("Executando Algoritmo de Jarník-Prim...");
            // Implementação do Algoritmo de Jarník-Prim
        }

        static void OrdenacaoTopologica(int vertices, int arestas)
        {
            Console.WriteLine("Executando Ordenação Topológica...");
            // Implementação da Ordenação Topológica
        }

        static void AlgoritmoDeKruskal(int vertices, int arestas)
        {
            Console.WriteLine("Executando Algoritmo de Kruskal...");
            // Implementação do Algoritmo de Kruskal
        }

        static void AlgoritmoDeFleury(int vertices, int arestas)
        {
            Console.WriteLine("Executando Algoritmo de Fleury...");
            // Implementação do Algoritmo de Fleury
        }

        static void AlgoritmoDeKonigEgervary(int vertices, int arestas)
        {
            Console.WriteLine("Executando Algoritmo de König-Egerváry...");
            // Implementação do Algoritmo de König-Egerváry
        }

        static void AlgoritmoGulosoDeColoracao(int vertices, int arestas)
        {
            Console.WriteLine("Executando Algoritmo Guloso de Coloração...");
            // Implementação do Algoritmo Guloso de Coloração
        }

        static void AlgoritmoDeWelshPowell(int vertices, int arestas)
        {
            Console.WriteLine("Executando Algoritmo de Welsh-Powell...");
            // Implementação do Algoritmo de Welsh-Powell
        }

        static void AlgoritmoDeBrelaz(int vertices, int arestas)
        {
            Console.WriteLine("Executando Algoritmo de Brélaz...");
            // Implementação do Algoritmo de Brélaz
        }

        static void AlgoritmoDeKosaraju(int vertices, int arestas)
        {
            Console.WriteLine("Executando Algoritmo de Kosaraju...");
            // Implementação do Algoritmo de Kosaraju
        }

        static void AlgoritmoDeKahn(int vertices, int arestas)
        {
            Console.WriteLine("Executando Algoritmo de Kahn...");
            // Implementação do Algoritmo de Kahn
        }

        static void AlgoritmoDeBellmanFord(int vertices, int arestas)
        {
            Console.WriteLine("Executando Algoritmo de Bellman-Ford...");
            // Implementação do Algoritmo de Bellman-Ford
        }

        static void AlgoritmoDeFordFulkerson(int vertices, int arestas)
        {
            Console.WriteLine("Executando Algoritmo de Ford-Fulkerson...");
            // Implementação do Algoritmo de Ford-Fulkerson
        }

    }
}
