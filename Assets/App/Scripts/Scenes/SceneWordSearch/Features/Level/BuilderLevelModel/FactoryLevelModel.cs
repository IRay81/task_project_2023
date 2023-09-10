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
            if (words != null)
            {
                Dictionary<char, int> letterCount = GetLetterCount(words);

                List<char> listChars = GetListFromLetterCount(letterCount);

                return listChars;
            }
            else
            {
                return null;

                //throw new Exception();
            }
        }

        private Dictionary<char, int> GetLetterCount(List<string> words)
        {
            Dictionary<char, int> letterCount = new Dictionary<char, int>();

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

                    if (letterCount.TryGetValue(word[i], out int lastCount))
                    {
                        if (lastCount < count)
                        {
                            letterCount[word[i]] = count;
                        }
                    }
                    else
                    {
                        letterCount.Add(word[i], count);
                    }
                }
            }

            return letterCount;
        }

        private List<char> GetListFromLetterCount(Dictionary<char, int> letterCount)
        {
            List<char> listChars = new();

            foreach (KeyValuePair<char, int> keyValuePair in letterCount)
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