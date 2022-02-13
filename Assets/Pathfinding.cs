using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class Pathfinding : MonoBehaviour
{

	public Transform seeker, target;
	Grid grid;
	int cntUCS = 0;
	int cntBFS = 0;
	int cntDFS = 0;
	int cntAStar = 0;
	int cntAStarManhattan = 0;
	int cntAStarEuclidean = 0;
	int fringeAStar = 0;
	int fringeAStarEuclidean = 0;
	int fringeAStarManhattan = 0;
	int fringeBFS = 0;
	int fringeDFS = 0;
	int fringeUCS = 0;
	

	void Awake()
	{
		grid = GetComponent<Grid>();
	}

	void Update()
	{
		
		if (seeker.hasChanged || target.hasChanged)
		{
			var timer = new System.Diagnostics.Stopwatch();
			
			timer.Start();
			FindPathAStar(seeker.position, target.position);
			timer.Stop();

			Debug.Log($"Time taken by A* algorithm: {timer.ElapsedMilliseconds} ms, path length: {cntAStar}, Fringe Memory: {fringeAStar}");
			timer.Reset();

			timer.Start();
			FindPathAStarManhattan(seeker.position, target.position);
			timer.Stop();
			Debug.Log($"Time taken by A* (With Manhattan Heuristic) algorithm: {timer.ElapsedMilliseconds} ms, path length: {cntAStarManhattan}, Fringe Memory: {fringeAStarManhattan}");

			timer.Reset();

			timer.Start();
			FindPathAStarEuclidean(seeker.position, target.position);
			timer.Stop();
			Debug.Log($"Time taken by A* (With Euclidean Heuristic) algorithm: {timer.ElapsedMilliseconds} ms, path length: {cntAStarEuclidean}, Fringe Memory: {fringeAStarEuclidean}");

			timer.Reset();

			timer.Start();
			FindPathBFS(seeker.position, target.position);
			timer.Stop();
			Debug.Log($"Time taken by BFS algorithm: {timer.ElapsedMilliseconds} ms, path length: {cntBFS}, Fringe Memory: {fringeBFS}");

			timer.Reset();

			timer.Start();
			FindPathUCS(seeker.position, target.position);
			timer.Stop();
			Debug.Log($"Time taken by UCS algorithm: {timer.ElapsedMilliseconds} ms, path length: {cntUCS}, Fringe Memory: {fringeUCS}");

			timer.Reset();

			timer.Start();
			FindPathDFS(seeker.position, target.position);
			timer.Stop();
			Debug.Log($"Time taken by DFS algorithm: {timer.ElapsedMilliseconds} ms, path length: {cntDFS}, Fringe Memory: {fringeDFS}");
			timer.Reset();
		}

	}

	void FindPathAStar(Vector3 startPos, Vector3 targetPos)
	{
		fringeAStar = 0;
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		openSet.Add(startNode);
		fringeAStar++;
		while (openSet.Count > 0)
		{
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i++)
			{
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost)
				{
					if (openSet[i].hCost < node.hCost)
						node = openSet[i];
				}
			}

			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode)
			{
				RetracePathAStar(startNode, targetNode);
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node))
			{
				if (!neighbour.walkable || closedSet.Contains(neighbour))
				{
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistance(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
				{
					neighbour.gCost = newCostToNeighbour;
					neighbour.hCost = GetDistance(neighbour, targetNode);
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
                    {
						openSet.Add(neighbour);
						fringeAStar++;
					}
				}
			}
		}
	}
	void FindPathAStarManhattan(Vector3 startPos, Vector3 targetPos)
	{
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);
		fringeAStarManhattan = 0;

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		openSet.Add(startNode);
		fringeAStarManhattan++;

		while (openSet.Count > 0)
		{
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i++)
			{
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost)
				{
					if (openSet[i].hCost < node.hCost)
						node = openSet[i];
				}
			}

			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode)
			{
				RetracePathAStarManhattan(startNode, targetNode);
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node))
			{
				if (!neighbour.walkable || closedSet.Contains(neighbour))
				{
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistanceManhattan(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
				{
					neighbour.gCost = newCostToNeighbour;
					neighbour.hCost = GetDistanceManhattan(neighbour, targetNode);
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
                    {
						openSet.Add(neighbour);
						fringeAStarManhattan++;
					}
						

				}
			}
		}
	}
	void FindPathAStarEuclidean(Vector3 startPos, Vector3 targetPos)
	{
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);
		fringeAStarEuclidean = 0;

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		openSet.Add(startNode);
		fringeAStarEuclidean++;

		while (openSet.Count > 0)
		{
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i++)
			{
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost)
				{
					if (openSet[i].hCost < node.hCost)
						node = openSet[i];
				}
			}

			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode)
			{
				RetracePathAStarEuclidean(startNode, targetNode);
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node))
			{
				if (!neighbour.walkable || closedSet.Contains(neighbour))
				{
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistanceEuclidean(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
				{
					neighbour.gCost = newCostToNeighbour;
					neighbour.hCost = GetDistanceEuclidean(neighbour, targetNode);
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
                    {
						openSet.Add(neighbour);
						fringeAStarEuclidean++;
					}
						

				}
			}
		}
	}
	void FindPathDFS(Vector3 startPos, Vector3 targetPos)
	{
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);
		Stack<Node> stack = new Stack<Node>();
		HashSet<Node> explored = new HashSet<Node>();
		fringeDFS = 0;
		stack.Push(startNode);
		fringeDFS++;

		while (stack.Count != 0)
		{
			Node current = stack.Pop();
			
			explored.Add(current);

			foreach (Node neighbor in grid.GetNeighbours(current))
			{
				if (!neighbor.walkable || explored.Contains(neighbor))
					continue;
				if (neighbor.walkable & !explored.Contains(neighbor))
				{
					if (neighbor == targetNode)
					{
						RetracePathDFS(startNode, targetNode);
						return;
					}
					neighbor.parent = current;
					stack.Push(neighbor);
					fringeDFS++;
				}
			}
		}
	}
	void FindPathBFS(Vector3 startPos, Vector3 targetPos)
	{

		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		Queue<Node> queue = new Queue<Node>();
		HashSet<Node> exploredNodes = new HashSet<Node>();
		HashSet<Node> nodeParents = new HashSet<Node>();
		IList<Node> walkablenodes = new List<Node>();
		fringeBFS = 0;

		queue.Enqueue(startNode);
		fringeBFS++;

		while (queue.Count != 0)
		{
			Node currentNode = queue.Dequeue();
			if (currentNode == targetNode)
			{
				RetracePathBFS(startNode, targetNode);
				return;
			}

			exploredNodes.Add(currentNode);

			foreach (Node neighbour in grid.GetNeighbours(currentNode))
			{
				if (neighbour.walkable)
				{
					walkablenodes.Add(neighbour);
				}
			}

			foreach (Node n in walkablenodes)
			{
				if (!exploredNodes.Contains(n))
				{
					exploredNodes.Add(n);

					nodeParents.Add(n);

					queue.Enqueue(n);
					fringeBFS++;
				}
			}
		}

	}
	void FindPathUCS(Vector3 startPos, Vector3 targetPos)
	{
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);
		fringeUCS = 0;

		List<Node> unexplored = new List<Node>();
		HashSet<Node> explored = new HashSet<Node>();
		unexplored.Add(startNode);
		fringeUCS++;

		while (unexplored.Count > 0)
		{
			Node node = unexplored[0];
			for (int i = 1; i < unexplored.Count; i++)
			{
				if (unexplored[i].fCost <= node.fCost)
				{
					if (unexplored[i].hCost < node.hCost)
						node = unexplored[i];
				}
			}

			unexplored.Remove(node);
			explored.Add(node);

			if (node == targetNode)
			{
				RetracePathUCS(startNode, targetNode);
				return;
			}
			foreach (Node neighbor in grid.GetNeighbours(node))
			{
				if (!neighbor.walkable || explored.Contains(neighbor))
					continue;
				int updatedCost = node.gCost;
				if (updatedCost < neighbor.gCost || !unexplored.Contains(neighbor))
				{
					neighbor.gCost = updatedCost;
					neighbor.hCost = 0;
					neighbor.parent = node;

					if (!unexplored.Contains(neighbor))
                    {
						unexplored.Add(neighbor);
						fringeUCS++;
					}
						
				}
			}
		}

	}
	void RetracePathDFS(Node startNode, Node endNode)
	{
		List<Node> route = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode)
		{
			route.Add(currentNode);
			currentNode = currentNode.parent;
		}
		route.Reverse();
		this.cntDFS = route.Count;
		grid.pathDFS = route;
	}
	void RetracePathUCS(Node startNode, Node endNode)
	{
		List<Node> route = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode)
		{
			route.Add(currentNode);
			currentNode = currentNode.parent;
		}
		route.Reverse();
		this.cntUCS = route.Count;
		grid.pathUCS = route;
	}
	void RetracePathAStar(Node startNode, Node endNode)
	{
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode)
		{
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();

		this.cntAStar = path.Count;
		grid.path = path;

	}
	void RetracePathAStarManhattan(Node startNode, Node endNode)
	{
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode)
		{
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();

		this.cntAStarManhattan = path.Count;
		grid.pathManhattan = path;

	}
	void RetracePathAStarEuclidean(Node startNode, Node endNode)
	{
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode)
		{
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();

		this.cntAStarEuclidean = path.Count;
		grid.pathEuclidean = path;

	}
	void RetracePathBFS(Node startNode, Node endNode)
	{
		List<Node> pathBFS = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode)
		{
			pathBFS.Add(currentNode);
			currentNode = currentNode.parent;
		}
		pathBFS.Reverse();

		this.cntBFS = pathBFS.Count;
		grid.pathBFS = pathBFS;

	}

	int GetDistance(Node nodeA, Node nodeB)
	{
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

		if (dstX > dstY)
			return 14 * dstY + 10 * (dstX - dstY);
		return 14 * dstX + 10 * (dstY - dstX);
	}
	int GetDistanceManhattan(Node nodeA, Node nodeB)
	{
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);
		//Debug.Log($"Manhattan Distance: { dstX + dstY} ");
		return dstX + dstY;
	}

	int GetDistanceEuclidean(Node nodeA, Node nodeB)
	{
		float x = Mathf.Pow(nodeB.gridX - nodeA.gridX, 2);
		float y = Mathf.Pow(nodeB.gridY - nodeA.gridY, 2);
		int result = (int)(Mathf.Sqrt(x + y));
		//Debug.Log($"Euclidean Distance: {result} ");
		return (int)( result);
	}
}