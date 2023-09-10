using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using App.Scripts.Scenes.SceneFillwords.Features.FillwordModels;
using UnityEngine;

namespace App.Scripts.Scenes.SceneFillwords.Features.ProviderLevel
{
    public class ProviderFillwordLevel : IProviderFillwordLevel
    {
        private readonly Regex _wordInLevelRegex = new(@"\d+\s(\d;?)+");
        private string[] _words;
        private string[] _levels;
        private bool _isFirstLoad = true;

        public GridFillWords LoadModel(int index)
        {
            if (_isFirstLoad)
            {
                _isFirstLoad = false;

                LoadResource();
            }

            if (index >= _levels.Length) //Исключение, если не удалось загрузить уровни.
            {
                throw new Exception();
            }

            Dictionary<int, int[]> wordsPositionsPair = new();
            int numbersCount = 0;

            MatchCollection wordsInLevelCollection = _wordInLevelRegex.Matches(_levels[index]);

            foreach (Match match in wordsInLevelCollection)
            {
                string[] stringPair = match.Value.Split(" ");
                string[] numbersString = stringPair[1].Split(";");

                int[] numbersInt = new int[numbersString.Length];

                for (int i = 0; i < numbersInt.Length; i++)
                {
                    numbersInt[i] = int.Parse(numbersString[i]);
                }

                wordsPositionsPair.Add(int.Parse(stringPair[0]), numbersInt);
                numbersCount += numbersInt.Length;
            }

            int size = (int)Math.Sqrt(numbersCount);

            if (size == Math.Sqrt(numbersCount)) //Проверка на квадратную матрицу
            {
                if (LevelCheck(wordsPositionsPair))
                {
                    GridFillWords gridFillWords = FillGrid(wordsPositionsPair, size);

                    return gridFillWords;
                }
                else
                {
                    return LoadModel(++index);
                }
            }
            else
            {
                return LoadModel(++index);
            }
        }

        /// <summary>
        /// Чтение из ресурсов файлов при первой загрузке уровня.
        /// </summary>
        private void LoadResource()
        {
            TextAsset asset = Resources.Load<TextAsset>("Fillwords/words_list");

            if (asset == null)
            {
                throw new Exception();
            }
            else
            {
                string allWords = asset.text;
                _words = allWords.Split(Environment.NewLine);

                asset = Resources.Load<TextAsset>("Fillwords/pack_0");

                if (asset == null)
                {
                    throw new Exception();
                }
                else
                {
                    string allLevels = asset.text;
                    _levels = allLevels.Split(Environment.NewLine);
                }
            }
        }

        /// <summary>
        /// Проверка уровня на валидность.
        /// </summary>
        /// <param name="totalNumbers">Все индексы слов.</param>
        /// <param name="wordsPositionsPair">Dictionary, где key - номер слова в словаре, а value - массив с индексами позиций букв в матрице.</param>
        /// <returns></returns>
        private bool LevelCheck(Dictionary<int, int[]> wordsPositionsPair)
        {
            List<int> totalNumbers = new List<int>();

            foreach (int[] g in wordsPositionsPair.Values)
            {
                totalNumbers.AddRange(g);
            }

            if (totalNumbers.Max() >= totalNumbers.Count) //Проверка на возможность индекса в матрице
            {
                return false;
            }

            for (int i = 0; i < totalNumbers.Count - 1; i++) //Проверка на совпадение индексов
            {
                for (int j = i + 1; j < totalNumbers.Count; j++)
                {
                    if (totalNumbers[i] == totalNumbers[j])
                    {
                        return false;
                    }
                }
            }

            foreach (KeyValuePair<int, int[]> pair in wordsPositionsPair) //Проверка на совпадение длин слова в словаре и уровне
            {
                if (_words[pair.Key].Length != pair.Value.Length)
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// Метод для заполнения матрицы словами.
        /// </summary>
        /// <param name="wordsPositionsPair">Dictionary, где key - номер слова в словаре, а value - массив с индексами позиций букв в матрице.</param>
        /// <param name="size">Размер матрицы.</param>
        /// <returns>Заполненная матрица GridFillWords</returns>
        private GridFillWords FillGrid(Dictionary<int, int[]> wordsPositionsPair, int size)
        {
            GridFillWords gridFillWords = new GridFillWords(new Vector2Int(size, size));
            CharGridModel charGridModel;

            foreach (KeyValuePair<int, int[]> pair in wordsPositionsPair)
            {
                for (int ind = 0; ind < pair.Value.Length; ind++)
                {
                    charGridModel = new CharGridModel(_words[pair.Key][ind]);

                    int i = pair.Value[ind] / size;
                    int j = pair.Value[ind] % size;

                    gridFillWords.Set(i, j, charGridModel);
                }

            }

            return gridFillWords;
        }
    }
}