using System;
using System.Collections.Generic;
using App.Scripts.Libs.Factory;
using App.Scripts.Scenes.SceneWordSearch.Features.Level.Models.Level;

namespace App.Scripts.Scenes.SceneWordSearch.Features.Level.BuilderLevelModel
{
    public class FactoryLevelModel : IFactory<LevelModel, LevelInfo, int>
    {
        public LevelModel Create(LevelInfo value, int levelNumber)
        {
            var model = new LevelModel();

            model.LevelNumber = levelNumber;

            model.Words = value.words;
            model.InputChars = BuildListChars(value.words);

            return model;
        }

        private List<char> BuildListChars(List<string> words)
        {
            Dictionary<char, int>  lettersCount = new Dictionary<char, int>();

            foreach (var word in words)
            {
                for (int i = 0; i < word.Length; i++)
                {
                    int count = 1;

                    for (int j = i + 1; j < word.Length; j++)
                    {
                        if (word[i].CompareTo(word[j]) == 0)
                        {
                            count++;
                        }
                    }

                    if (lettersCount.TryGetValue(word[i], out int lastCount))
                    {
                        if (lastCount < count)
                        {
                            lettersCount[word[i]] = count;
                        }
                    }
                    else
                    {
                        lettersCount.Add(word[i], count);
                    }
                }
            }

            List<char> listChars = new List<char>();

            foreach (KeyValuePair<char, int> keyValuePair in lettersCount)
            {
                for (int i = 0; i < keyValuePair.Value; i++)
                {
                    listChars.Add(keyValuePair.Key);
                }
            }

            return listChars;
        }
    }
}