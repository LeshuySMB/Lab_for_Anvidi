#include <iostream>
#include <vector>
#include <queue>
#include <tuple>
#include <limits>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <functional>

using namespace std;

// Структура для представления ребра графа
struct Edge {
    int to;              // Конечная вершина
    double travel_time;  // Время прохождения по ребру
};

// Функция для чтения вершин из файла
vector<pair<double, double>> parse_vertices(ifstream &file, int num_vertices) {
    vector<pair<double, double>> vertices(num_vertices);
    for (int i = 0; i < num_vertices; ++i) {
        double x, y;
        file >> x >> y;  // Считывание координат вершины
        vertices[i] = {x, y};
    }
    return vertices;
}

// Функция для чтения рёбер из файла и построения графа
vector<vector<Edge>> parse_edges(ifstream &file, int num_vertices, int num_edges) {
    vector<vector<Edge>> graph(num_vertices);
    for (int i = 0; i < num_edges; ++i) {
        int n1, k1;
        double p1, v1;
        file >> n1 >> k1 >> p1 >> v1;  // Считывание данных о ребре
        double travel_time = (100 / (100 - v1)) * p1;  // Вычисление времени прохождения
        graph[n1].push_back({k1, travel_time});  // Добавление ребра в граф
        graph[k1].push_back({n1, travel_time});  // Граф неориентированный
    }
    return graph;
}

// Реализация алгоритма Дейкстры для поиска самого короткого пути
pair<vector<int>, double> dijkstra(const vector<vector<Edge>>& graph, int start, int end) {
    int n = graph.size();
    vector<double> distances(n, numeric_limits<double>::infinity());  // Вектор расстояний
    vector<int> predecessors(n, -1);  // Вектор предшественников
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> queue;  // Очередь приоритетов

    distances[start] = 0;  // Расстояние до стартовой вершины
    queue.push({0, start});  // Добавление стартовой вершины в очередь

    while (!queue.empty()) {
        double current_distance = queue.top().first;
        int current_vertex = queue.top().second;
        queue.pop();

        if (current_distance > distances[current_vertex]) {
            continue;
        }

        for (const auto& edge : graph[current_vertex]) {
            int neighbor = edge.to;
            double weight = edge.travel_time;
            double distance = current_distance + weight;

            if (distance < distances[neighbor]) {
                distances[neighbor] = distance;
                predecessors[neighbor] = current_vertex;
                queue.push({distance, neighbor});
            }
        }
    }

    vector<int> path;  // Вектор для хранения пути
    for (int at = end; at != -1; at = predecessors[at]) {
        path.push_back(at);
    }
    reverse(path.begin(), path.end());  // Обратный порядок, чтобы путь был от start до end

    return {path, distances[end]};
}

int main() {
    string file_path = "city_map.txt";  // Путь к файлу с данными
    ifstream file(file_path);

    if (!file.is_open()) {
        cerr << "Ошибка при открытии файла" << endl;
        return 1;
    }

    int num_vertices, num_edges;
    file >> num_vertices;  // Чтение числа вершин
    auto vertices = parse_vertices(file, num_vertices);  // Считывание вершин

    file >> num_edges;  // Чтение числа рёбер
    auto graph = parse_edges(file, num_vertices, num_edges);  // Считывание рёбер и построение графа

    int start = 0;  // Начальная вершина (можно заменить на нужную)
    int end = 5;    // Конечная вершина (можно заменить на нужную)

    auto result = dijkstra(graph, start, end);  // Поиск самого быстрого маршрута
    vector<int> path = result.first;
    double total_time = result.second;

    cout << "Самый быстрый маршрут: ";
    for (int vertex : path) {
        cout << vertex << " ";
    }
    cout << endl;
    cout << "Общее время в пути: " << total_time << endl;

    return 0;
}
