using System;
using System.IO;
using System.Collections.Generic;

namespace GrafosApp {
    public class Program {
        public static string arquivo = "./Grafo.txt";

        static void processarGrafo(string caminhoArquivo) {

            try {
                if (!File.Exists(caminhoArquivo)) {
                    Console.WriteLine("Erro: O arquivo '{caminhoArquivo}' não foi encontrado.");
                }

                string[] grafo = File.ReadAllLines(caminhoArquivo);

                if (grafo.Length == 0) {
                    Console.WriteLine("Erro: O arquivo '{caminhoArquivo}' está vazio.");
                }
                Console.WriteLine("Seu Grafo:");
                for(int i = 0; i < grafo.Length; i++) {
                    Console.WriteLine(grafo[i]);
                }

                int opcao = int.Parse(grafo[0]);
                string infos = grafo[1];
                string[] split = infos.Split(' ');
                int vertices = int.Parse(split[0]);
                int arestas = int.Parse(split[1]);
                switch (opcao) {
                    case 1:
                        BuscaEmProfundidade(vertices, arestas);
                        break;
                    case 2:
                        BuscaEmLargura(vertices, arestas);
                        break;
                    case 3:
                        AlgoritmoDeDijkstra(vertices, arestas);
                        break;
                    case 4:
                        AlgoritmoDePrim(vertices, arestas);
                        break;
                    case 5:
                        OrdenacaoTopologica(vertices, arestas);
                        break;
                    case 6:
                        AlgoritmoDeKruskal(vertices, arestas);
                        break;
                    case 7:
                        AlgoritmoDeFleury(vertices, arestas);
                        break;
                    case 8:
                        AlgoritmoDeKonigEgervary(vertices, arestas);
                        break;
                    case 9:
                        AlgoritmoGulosoDeColoracao(vertices, arestas);
                        break;
                    case 10:
                        AlgoritmoDeWelshPowell(vertices, arestas);
                        break;
                    case 11:
                        AlgoritmoDeBrelaz(vertices, arestas);
                        break;
                    case 12:
                        AlgoritmoDeKosaraju(vertices, arestas);
                        break;
                    case 13:
                        AlgoritmoDeKahn(vertices, arestas);
                        break;
                    case 14:
                        AlgoritmoDeBellmanFord(vertices, arestas);
                        break;
                    case 15:
                        AlgoritmoDeFordFulkerson(vertices, arestas);
                        break;
                    default:
                        Console.WriteLine("Opção inválida. Tente novamente.");
                        break;
                }
            } catch (IOException e) {
                Console.WriteLine("Ocorreu um erro ao ler o arquivo: {e.Message}");
            } catch (UnauthorizedAccessException) {
                Console.WriteLine("Erro: Sem permissão para acessar o arquivo '{caminhoArquivo}'.");
            }
        }

        public static void Main(string[] args){
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
        static void BuscaEmProfundidade(int vertices, int arestas) {
            Console.WriteLine("Executando Busca em Profundidade (DFS)...");
            // Implementação do Algoritmo de Busca em Profundidade
        }

        static void BuscaEmLargura(int vertices, int arestas) {
            Console.WriteLine("Executando Busca em Largura (BFS)...");
            // Implementação do Algoritmo de Busca em Largura
        }

        static void AlgoritmoDeDijkstra(int vertices, int arestas) {
            Console.WriteLine("Executando Algoritmo de Dijkstra...");
            // Implementação do Algoritmo de Dijkstra
        }

        static void AlgoritmoDePrim(int vertices, int arestas) {
            Console.WriteLine("Executando Algoritmo de Jarník-Prim...");
            // Implementação do Algoritmo de Jarník-Prim
        }

        static void OrdenacaoTopologica(int vertices, int arestas) {
            Console.WriteLine("Executando Ordenação Topológica...");
            // Implementação da Ordenação Topológica
        }

        static void AlgoritmoDeKruskal(int vertices, int arestas) {
            Console.WriteLine("Executando Algoritmo de Kruskal...");
            // Implementação do Algoritmo de Kruskal
        }

        static void AlgoritmoDeFleury(int vertices, int arestas) {
            Console.WriteLine("Executando Algoritmo de Fleury...");
            // Implementação do Algoritmo de Fleury
        }

        static void AlgoritmoDeKonigEgervary(int vertices, int arestas) {
            Console.WriteLine("Executando Algoritmo de König-Egerváry...");
            // Implementação do Algoritmo de König-Egerváry
        }

        static void AlgoritmoGulosoDeColoracao(int vertices, int arestas) {
            Console.WriteLine("Executando Algoritmo Guloso de Coloração...");
            // Implementação do Algoritmo Guloso de Coloração
        }

        static void AlgoritmoDeWelshPowell(int vertices, int arestas) {
            Console.WriteLine("Executando Algoritmo de Welsh-Powell...");
            // Implementação do Algoritmo de Welsh-Powell
        }

        static void AlgoritmoDeBrelaz(int vertices, int arestas) {
            Console.WriteLine("Executando Algoritmo de Brélaz...");
            // Implementação do Algoritmo de Brélaz
        }

        static void AlgoritmoDeKosaraju(int vertices, int arestas) {
            Console.WriteLine("Executando Algoritmo de Kosaraju...");
            // Implementação do Algoritmo de Kosaraju
        }

        static void AlgoritmoDeKahn(int vertices, int arestas) {
            Console.WriteLine("Executando Algoritmo de Kahn...");
            // Implementação do Algoritmo de Kahn
        }

        static void AlgoritmoDeBellmanFord(int vertices, int arestas) {
            Console.WriteLine("Executando Algoritmo de Bellman-Ford...");
            // Implementação do Algoritmo de Bellman-Ford
        }

        static void AlgoritmoDeFordFulkerson(int vertices, int arestas) {
            Console.WriteLine("Executando Algoritmo de Ford-Fulkerson...");
            // Implementação do Algoritmo de Ford-Fulkerson
        }
    }
}