using System;
using System.Collections.Generic;
using System.Linq;
using App.Scripts.Scenes.SceneChess.Features.ChessField.GridMatrix;
using App.Scripts.Scenes.SceneChess.Features.ChessField.Types;
using UnityEngine;

namespace App.Scripts.Scenes.SceneChess.Features.GridNavigation.Navigator
{
    public class ChessGridNavigator : IChessGridNavigator
    {
        private const int MaxIterations = 1000000;

        private static readonly Vector2Int[] _whitePonMoves =
        {
            new Vector2Int(0, 1)
        };

        private static readonly Vector2Int[] _blackPonMoves =
        {
            new Vector2Int(0, -1)
        };

        private static readonly Vector2Int[] _knightMoves = {
            new Vector2Int(1, 2),
            new Vector2Int(2, 1),
            new Vector2Int(2, -1),
            new Vector2Int(1, -2),
            new Vector2Int(-1, -2),
            new Vector2Int(-2, -1),
            new Vector2Int(-2, 1),
            new Vector2Int(-1, 2)
        };

        private static readonly Vector2Int[] _bishopMoves = {
            new Vector2Int(1, 1),
            new Vector2Int(1, -1),
            new Vector2Int(-1, 1),
            new Vector2Int(-1, -1)
        };

        private static readonly Vector2Int[] _rookMoves = {
            new Vector2Int(1, 0),
            new Vector2Int(0, 1),
            new Vector2Int(-1, 0),
            new Vector2Int(0, -1)
        };

        private static readonly Vector2Int[] _kingAndQueenMoves = {
            new Vector2Int(1, 0),
            new Vector2Int(0, 1),
            new Vector2Int(-1, 0),
            new Vector2Int(0, -1),
            new Vector2Int(1, 1),
            new Vector2Int(1, -1),
            new Vector2Int(-1, 1),
            new Vector2Int(-1, -1)
        };

        public List<Vector2Int> FindPath(ChessUnitType unit, Vector2Int from, Vector2Int to, ChessGrid grid)
        {
            List<Vector2Int> obstacles = grid.Pieces.Select(p => p.CellPosition).ToList();

            ChessUnitColor color = grid.Get(from).PieceModel.Color;

            List<Vector2Int> pathList = CalculatePath(from, to, obstacles, unit, color);

            if (unit is ChessUnitType.Queen || unit is ChessUnitType.Rook || unit is ChessUnitType.Bishop)
            {
                pathList = TrimPath(pathList);
            }

            pathList.Remove(from);
            pathList.Reverse();

            return pathList;
        }

        /// <summary>
        /// Убираем соседние ходы для слона, ладьи и ферзя.
        /// </summary>
        /// <param name="pathList">Исходный путь.</param>
        /// <returns>Урезанный путь фигуры.</returns>
        /// <exception cref="ArgumentNullException"></exception>
        private List<Vector2Int> TrimPath(List<Vector2Int> pathList)
        {
            if (pathList == null)
            {
                throw new ArgumentNullException(nameof(pathList));
            }

            List<Vector2Int> pathToRemove = new List<Vector2Int>();

            for (int i = 0; i < pathList.Count - 2; i++)
            {
                if (pathList[i] - pathList[i + 1] == pathList[i + 1] - pathList[i + 2])
                {
                    pathToRemove.Add(pathList[i + 1]);
                }
            }

            foreach (Vector2Int path in pathToRemove)
            {
                pathList.Remove(path);

            }

            return pathList;
        }

        /// <summary>
        /// Смещение в сторону направления движения (для ферзя, ладьи и слона).
        /// </summary>
        /// <param name="position">Начальная позиция.</param>
        /// <param name="move">Направление движения.</param>
        /// <param name="shift">Смещение.</param>
        /// <returns>Конечная позиция.</returns>
        private Vector2Int Shift(Vector2Int position, Vector2Int move, int shift)
        {
            return position + shift * move;
        }

        /// <summary>
        /// Метод нахождения возможных позиций фигуры на следующий ход.
        /// </summary>
        /// <param name="position">Текущая позиция.</param>
        /// <param name="obstacles">Препятствия на доске.</param>
        /// <param name="type">Тип фигуры.</param>
        /// <param name="unitColor">Цвет фигуры (для пешки).</param>
        /// <returns></returns>
        private List<Vector2Int> FindPossibleMoves(Vector2Int position, List<Vector2Int> obstacles, ChessUnitType type, ChessUnitColor unitColor)
        {
            List<Vector2Int> vectors = new();
            List<Vector2Int> dir;

            switch (type)
            {
                case ChessUnitType.Pon:
                    if (unitColor is ChessUnitColor.White)
                    {
                        dir = new List<Vector2Int>(_whitePonMoves);

                    }
                    else
                    {
                        dir = new List<Vector2Int>(_blackPonMoves);
                    }
                    break;
                case ChessUnitType.King:
                case ChessUnitType.Queen:
                    dir = new List<Vector2Int>(_kingAndQueenMoves);
                    break;
                case ChessUnitType.Rook:
                    dir = new List<Vector2Int>(_rookMoves);
                    break;
                case ChessUnitType.Knight:
                    dir = new List<Vector2Int>(_knightMoves);
                    break;
                default:
                    dir = new List<Vector2Int>(_bishopMoves);
                    break;
            }

            int shift = 1;

            if (type is ChessUnitType.Queen || type is ChessUnitType.Bishop || type is ChessUnitType.Rook)
            {
                while (dir.Count > 0)
                {
                    AddMoves(in dir, in vectors, obstacles, position, shift);

                    shift++;
                }
            }
            else
            {
                AddMoves(in dir, in vectors, obstacles, position, shift);
            }
            
            return vectors;
        }

        /// <summary>
        /// Метод добавления возможных позиций.
        /// </summary>
        /// <param name="dir">Возможные направления перемещения.</param>
        /// <param name="vectors">Список возможных позиций.</param>
        /// <param name="obstacles">Препятствия на доске.</param>
        /// <param name="position">Текущая позиция.</param>
        /// <param name="shift">Смещение.</param>
        private void AddMoves(in List<Vector2Int> dir, in List<Vector2Int> vectors, List<Vector2Int> obstacles, Vector2Int position, int shift)
        {
            for (int i = 0; i < dir.Count; i++)
            {
                Vector2Int move = Shift(position, dir[i], shift);

                if (!obstacles.Contains(move) && move.x >= 0 && move.x < 8 && move.y >= 0 && move.y < 8)
                {
                    vectors.Add(move);
                }
                else
                {
                    dir.RemoveAt(i);
                    i--;
                }
            }
        }

        /// <summary>
        /// Метод поиска пути с использованием алгоритма поиска в ширину (BFC).
        /// </summary>
        /// <param name="start">Начальная клетка.</param>
        /// <param name="finish">Конечная клетка.</param>
        /// <param name="obstacles">Список препятствий.</param>
        /// <param name="type">Тип фигуры.</param>
        /// <param name="unitColor">Цвет фигуры (для пешек)</param>
        /// <returns>Список с ходами фигуры.</returns>
        private List<Vector2Int> CalculatePath(Vector2Int start, Vector2Int finish, List<Vector2Int> obstacles, ChessUnitType type, ChessUnitColor unitColor = ChessUnitColor.White)
        {
            int iterations = 0;

            Queue<Vector2Int> queue = new Queue<Vector2Int>();
            queue.Enqueue(start);

            List<Vector2Int> explored = new List<Vector2Int>
            {
                start
            };

            var parent = new Dictionary<Vector2Int, Vector2Int>();

            List<Vector2Int> possibleMoves;

            while (queue.Count > 0)
            {
                iterations++;
                if (iterations >= MaxIterations) return null;

                Vector2Int pos = queue.Dequeue();

                possibleMoves = FindPossibleMoves(pos, obstacles, type, unitColor);

                for (int j = 0; j < possibleMoves.Count; j++)
                {
                    Vector2Int move = possibleMoves[j];

                    if (explored.IndexOf(move) < 0)
                    {
                        parent[move] = pos;
                        explored.Add(move);

                        if (move == finish)
                        {
                            queue.Clear();
                            break;
                        }
                        else
                        {
                            queue.Enqueue(move);
                        }
                    }
                }
            }

            List<Vector2Int> pathList = new List<Vector2Int>();

            try
            {
                pathList.Add(finish);

                var current = parent[finish];
                while (!current.Equals(start))
                {
                    pathList.Add(current);
                    current = parent[current];
                } 

                pathList.Add(start);
            }
            catch
            {
                return null;
            }

            return pathList;
        }
    }
}