using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using App.Scripts.Scenes.SceneChess.Features.ChessField.GridMatrix;
using App.Scripts.Scenes.SceneChess.Features.ChessField.Piece;
using App.Scripts.Scenes.SceneChess.Features.ChessField.Types;
using DG.Tweening;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UIElements;
using UnityEngine.VFX;
using static App.Scripts.Scenes.SceneChess.Features.GridNavigation.Navigator.ChessGridNavigator;

namespace App.Scripts.Scenes.SceneChess.Features.GridNavigation.Navigator
{
    public class ChessGridNavigator : IChessGridNavigator
    {
        public List<Vector2Int> FindPath(ChessUnitType unit, Vector2Int from, Vector2Int to, ChessGrid grid)
        {
            List<Vector2Int> path = null;

            switch (unit)
            {
                case ChessUnitType.Pon:
                    path = FindPonPath(from, to, grid);
                    return path;
                case ChessUnitType.King:
                    path = FindPonPath(from, to, grid);
                    return path;
                case ChessUnitType.Queen:
                    path = FindPonPath(from, to, grid);
                    return path;
                case ChessUnitType.Rook:
                    path = FindPonPath(from, to, grid);
                    return path;
                case ChessUnitType.Knight:
                    path = FindPonPath(from, to, grid);
                    return path;
                case ChessUnitType.Bishop:
                    path = FindPonPath(from, to, grid);
                    return path;
            }

            return path;
        }

        private List<Vector2Int> FindPonPath(Vector2Int from, Vector2Int to, ChessGrid grid)
        {
            if (from.x != to.x)
            {
                return null;
            }

            ChessUnitColor color = grid.Get(from).PieceModel.Color;
            List<Vector2Int> path;

            if ((color is ChessUnitColor.White && from.y < to.y) || (color is ChessUnitColor.Black && from.y > to.y))
            {
                int offset = from.y < to.y ? 1 : -1;
                int moveCount = Math.Abs(from.y - to.y);

                path = new List<Vector2Int>();

                for (int i = 1; i <= moveCount; i++)
                {
                    int y = from.y + i * offset;

                    if (grid.Get(y, from.x) != null)
                    {
                        return null;
                    }
                    else
                    {
                        path.Add(new Vector2Int(from.x, y));
                    }
                }

                return path;
            }
            else
            {
                return null;
            }
        }

        //===================================================================

        /// <summary>
        /// Reusable A* path finder.
        /// </summary>
        public class Path
        {
            private const int MaxNeighbours = 8;
            private readonly PathNode[] neighbours = new PathNode[MaxNeighbours];

            private readonly int maxSteps;
            private readonly IBinaryHeap<Vector2Int, PathNode> frontier;
            private readonly HashSet<Vector2Int> ignoredPositions;
            private readonly List<Vector2Int> output;
            private readonly IDictionary<Vector2Int, Vector2Int> links;

            

            /// <summary>
            /// Creation of new path finder.
            /// </summary>
            /// <exception cref="ArgumentOutOfRangeException"></exception>
            public Path(int maxSteps = int.MaxValue, int initialCapacity = 0)
            {
                if (maxSteps <= 0)
                    throw new ArgumentOutOfRangeException(nameof(maxSteps));
                if (initialCapacity < 0)
                    throw new ArgumentOutOfRangeException(nameof(initialCapacity));

                this.maxSteps = maxSteps;
                var comparer = Comparer<PathNode>.Create((a, b) => b.EstimatedTotalCost.CompareTo(a.EstimatedTotalCost));
                frontier = new BinaryHeap<Vector2Int, PathNode>(comparer, a => a.Position, initialCapacity);
                ignoredPositions = new HashSet<Vector2Int>(initialCapacity);
                output = new List<Vector2Int>(initialCapacity);
                links = new Dictionary<Vector2Int, Vector2Int>(initialCapacity);
            }

            /// <summary>
            /// Calculate a new path between two points.
            /// </summary>
            /// <exception cref="ArgumentNullException"></exception>
            public bool Calculate(Vector2Int start, Vector2Int target, IReadOnlyCollection<Vector2Int> obstacles, ChessUnitType type,
                out IReadOnlyCollection<Vector2Int> path)
            {
                if (obstacles == null)
                    throw new ArgumentNullException(nameof(obstacles));

                if (!GenerateNodes(start, target, obstacles, type))
                {
                    path = Array.Empty<Vector2Int>();
                    return false;
                }

                output.Clear();
                output.Add(target);

                while (links.TryGetValue(target, out target)) output.Add(target);
                path = output;
                return true;
            }

            private bool GenerateNodes(Vector2Int start, Vector2Int target, IReadOnlyCollection<Vector2Int> obstacles, ChessUnitType type)
            {
                frontier.Clear();
                ignoredPositions.Clear();
                links.Clear();

                frontier.Enqueue(new PathNode(start, target, 0, type));
                ignoredPositions.UnionWith(obstacles);
                var step = 0;
                while (frontier.Count > 0 && step++ <= maxSteps)
                {
                    PathNode current = frontier.Dequeue();
                    ignoredPositions.Add(current.Position);

                    if (current.Position.Equals(target)) return true;

                    GenerateFrontierNodes(current, target, type);
                }

                // All nodes analyzed - no path detected.
                return false;
            }

            private void GenerateFrontierNodes(PathNode parent, Vector2Int target, ChessUnitType type)
            {
                neighbours.Fill(parent, target, type);
                foreach (PathNode newNode in neighbours)
                {
                    // Position is already checked or occupied by an obstacle.
                    if (ignoredPositions.Contains(newNode.Position)) continue;

                    // Node is not present in queue.
                    if (!frontier.TryGet(newNode.Position, out PathNode existingNode))
                    {
                        frontier.Enqueue(newNode);
                        links[newNode.Position] = parent.Position;
                    }

                    // Node is present in queue and new optimal path is detected.
                    else if (newNode.TraverseDistance < existingNode.TraverseDistance)
                    {
                        frontier.Modify(newNode);
                        links[newNode.Position] = parent.Position;
                    }
                }
            }
        }


    }

    internal interface IBinaryHeap<in TKey, T>
    {
        void Enqueue(T item);
        T Dequeue();
        void Clear();
        bool TryGet(TKey key, out T value);
        void Modify(T value);
        int Count { get; }
    }

    internal class BinaryHeap<TKey, T> : IBinaryHeap<TKey, T> where TKey : IEquatable<TKey>
    {
        private readonly IDictionary<TKey, int> map;
        private readonly IList<T> collection;
        private readonly IComparer<T> comparer;
        private readonly Func<T, TKey> lookupFunc;

        public BinaryHeap(IComparer<T> comparer, Func<T, TKey> lookupFunc, int capacity)
        {
            this.comparer = comparer;
            this.lookupFunc = lookupFunc;
            collection = new List<T>(capacity);
            map = new Dictionary<TKey, int>(capacity);
        }

        public int Count => collection.Count;

        public void Enqueue(T item)
        {
            collection.Add(item);
            int i = collection.Count - 1;
            map[lookupFunc(item)] = i;
            while (i > 0)
            {
                int j = (i - 1) / 2;

                if (comparer.Compare(collection[i], collection[j]) <= 0)
                    break;

                Swap(i, j);
                i = j;
            }
        }

        public T Dequeue()
        {
            if (collection.Count == 0) return default;

            T result = collection.First();
            RemoveRoot();
            map.Remove(lookupFunc(result));
            return result;
        }

        public void Clear()
        {
            collection.Clear();
            map.Clear();
        }

        public bool TryGet(TKey key, out T value)
        {
            if (!map.TryGetValue(key, out int index))
            {
                value = default;
                return false;
            }

            value = collection[index];
            return true;
        }

        public void Modify(T value)
        {
            if (!map.TryGetValue(lookupFunc(value), out int index))
                throw new KeyNotFoundException(nameof(value));

            collection[index] = value;
        }

        private void RemoveRoot()
        {
            collection[0] = collection.Last();
            map[lookupFunc(collection[0])] = 0;
            collection.RemoveAt(collection.Count - 1);

            var i = 0;
            while (true)
            {
                int largest = LargestIndex(i);
                if (largest == i)
                    return;

                Swap(i, largest);
                i = largest;
            }
        }

        private void Swap(int i, int j)
        {
            T temp = collection[i];
            collection[i] = collection[j];
            collection[j] = temp;
            map[lookupFunc(collection[i])] = i;
            map[lookupFunc(collection[j])] = j;
        }

        private int LargestIndex(int i)
        {
            int leftInd = 2 * i + 1;
            int rightInd = 2 * i + 2;
            int largest = i;

            if (leftInd < collection.Count && comparer.Compare(collection[leftInd], collection[largest]) > 0) largest = leftInd;

            if (rightInd < collection.Count && comparer.Compare(collection[rightInd], collection[largest]) > 0) largest = rightInd;

            return largest;
        }
    }

    internal readonly struct PathNode
    {
        public PathNode(Vector2Int position, Vector2Int target, double traverseDistance, ChessUnitType type)
        {
            Position = position;
            TraverseDistance = traverseDistance;
            double heuristicDistance = (position - target).DistanceEstimate(type);
            EstimatedTotalCost = traverseDistance + heuristicDistance;
        }

        public Vector2Int Position { get; }
        public double TraverseDistance { get; }
        public double EstimatedTotalCost { get; }
    }

    internal static class NodeExtensions
    {
        private static readonly (Vector2Int position, double cost)[] NeighboursKnightTemplate = {
            (new Vector2Int(1, 2), 1),
            (new Vector2Int(2, 1), 1),
            (new Vector2Int(2, -1), 1),
            (new Vector2Int(1, -2), 1),
            (new Vector2Int(-1, -2), 1),
            (new Vector2Int(-2, -1), 1),
            (new Vector2Int(-2, 1), 1),
            (new Vector2Int(-1, 2), 1)
        };

        private static readonly (Vector2Int position, double cost)[] NeighboursBishopTemplate = {
            (new Vector2Int(1, 1), 1),
            (new Vector2Int(1, -1), 1),
            (new Vector2Int(-1, 1), 1),
            (new Vector2Int(-1, -1), 1)
        };

        private static readonly (Vector2Int position, double cost)[] NeighboursRookTemplate = {
            (new Vector2Int(1, 0), 1),
            (new Vector2Int(0, 1), 1),
            (new Vector2Int(-1, 0), 1),
            (new Vector2Int(0, -1), 1)
        };

        private static readonly (Vector2Int position, double cost)[] NeighboursQueenAndKingTemplate = {
            (new Vector2Int(1, 0), 1),
            (new Vector2Int(0, 1), 1),
            (new Vector2Int(-1, 0), 1),
            (new Vector2Int(0, -1), 1),
            (new Vector2Int(1, 1), 1),
            (new Vector2Int(1, -1), 1),
            (new Vector2Int(-1, 1), 1),
            (new Vector2Int(-1, -1), 1)
        };

        public static void Fill(this PathNode[] buffer, PathNode parent, Vector2Int target, ChessUnitType type)
        {
            int i = 0;

            (Vector2Int position, double cost)[] NeighboursTemplate = null;

            switch (type)
            {
                case ChessUnitType.Knight:
                    NeighboursTemplate = NeighboursKnightTemplate;
                    break;
                case ChessUnitType.Bishop:
                    NeighboursTemplate = NeighboursBishopTemplate;
                    break;
                case ChessUnitType.Rook:
                    NeighboursTemplate = NeighboursRookTemplate;
                    break;
                case ChessUnitType.King:
                case ChessUnitType.Queen:
                    NeighboursTemplate = NeighboursQueenAndKingTemplate;
                    break;
            }

            foreach ((Vector2Int position, double cost) in NeighboursTemplate)
            {
                Vector2Int nodePosition = position + parent.Position;
                double traverseDistance = parent.TraverseDistance + cost;
                buffer[i++] = new PathNode(nodePosition, target, traverseDistance, type);
            }
        }

        /// <summary>
        /// Estimated path distance without obstacles.
        /// </summary>
        public static double DistanceEstimate(this Vector2Int vector, ChessUnitType type)
        {
            switch (type)
            {
                case ChessUnitType.Knight:
                    return FindShortestPath(vector, new Vector2Int(8, 8), NeighboursKnightTemplate[0]);
                case ChessUnitType.Bishop: //Дистанция для слона считается по диагонали.
                    return Math.Max(Math.Abs(vector.x), Math.Abs(vector.y));
                case ChessUnitType.Rook: //Дистанция для ладьи считается суммой координат.
                    return vector.x + vector.y;
                default: //Дистанция для короля и дамы считается суммой диагональным и линейных перемещений.
                    int linearSteps = Math.Abs(Math.Abs(vector.y) - Math.Abs(vector.x));
                    int diagonalSteps = Math.Max(Math.Abs(vector.y), Math.Abs(vector.x)) - linearSteps;
                    return linearSteps + diagonalSteps;
            }
        }

        /// <summary>
        /// Найти количество ходов для фигуры до точки назначения.
        /// </summary>
        /// <param name="to">Точка назначения.</param>
        /// <param name="gridSize">Размер доски.</param>
        /// <param name="directions">Возможные ходы.</param>
        /// <returns></returns>
        private static int FindShortestPath(Vector2Int to, Vector2Int gridSize, Vector2Int[] directions)
        {
            bool[][] visited = new bool[gridSize.x][];
            for (int i = 0; i < gridSize.x; i++)
            {
                visited[i] = new bool[gridSize.y];
            }

            Queue<int[]> queue = new Queue<int[]>();
            queue.Enqueue(new int[] { 0, 0 });

            visited[0][0] = true;

            int steps = 0;

            while (queue.Count > 0)
            {
                int size = queue.Count;

                for (int i = 0; i < size; i++)
                {
                    int[] currentPosition = queue.Dequeue();
                    int x = currentPosition[0];
                    int y = currentPosition[1];

                    if (x == to.x && y == to.y)
                    {
                        return steps;
                    }

                    for (int j = 0; j < directions.Length; j++)
                    {
                        int newX = x + directions[j].x;
                        int newY = y + directions[j].y;

                        if (newX >= 0 && newX < gridSize.x && newY >= 0 && newY < gridSize.y && !visited[newY][newX])
                        {
                            queue.Enqueue(new int[] { newX, newY });
                            visited[newY][newX] = true;
                        }
                    }
                }

                steps++;
            }

            return -1;  // кратчайший путь не найден
        }
    }
}