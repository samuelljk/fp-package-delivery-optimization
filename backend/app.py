from flask import Flask, request, jsonify
from flask_cors import CORS
from algorithms import tsp_brute_force, dijkstra, prim_mst, bfs, dfs

app = Flask(__name__)
CORS(app)  # Enable Cross-Origin Resource Sharing

@app.route('/tsp', methods=['POST'])
def tsp():
    data = request.json
    graph = data['graph']
    start = data['start']
    route, distance = tsp_brute_force(graph, start)
    return jsonify({"route": route, "distance": distance})

@app.route('/shortest_path', methods=['POST'])
def shortest_path():
    data = request.json
    graph = data['graph']
    start = data['start']
    end = data['end']
    path, distance = dijkstra(graph, start, end)
    return jsonify({"path": path, "distance": distance})

@app.route('/mst', methods=['POST'])
def mst():
    data = request.json
    graph = data['graph']
    mst_edges, total_cost = prim_mst(graph)
    return jsonify({"edges": mst_edges, "total_cost": total_cost})

@app.route('/bfs', methods=['POST'])
def bfs_route():
    data = request.json
    graph = data['graph']
    start = data['start']
    traversal = bfs(graph, start)
    return jsonify({"traversal": traversal})

@app.route('/dfs', methods=['POST'])
def dfs_route():
    data = request.json
    graph = data['graph']
    start = data['start']
    traversal = dfs(graph, start)
    return jsonify({"traversal": traversal})

if __name__ == '__main__':
    app.run(debug=True, port=8080)
