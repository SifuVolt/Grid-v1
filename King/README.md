# King — RLBot v5

This is a standalone RLBot v5 C# port of CodeRed's King bot.

- Strategy: recovered from the published `RLBotPack/King/Bot.dll` assembly.
- Runtime/utilities: vendored from [RedUtilsV5](https://github.com/ItsCodeRed/RedUtilsV5), with King-specific compatibility helpers in `Bot/KingCompat.cs`.
- Target framework: .NET 8.
- Build: `dotnet build .\\Bot\\Bot.csproj -c Release`
- RLBot v5 entrypoint: `bot.toml`.

The `Recovered` folder is reference-only decompilation output. The runnable v5 bot is in `Bot`, with the utility project in `RedUtils`.
