# Contributing

Pull requests are welcome.

## Contributor License Agreement

The project is licensed under the [AGPL-3.0](LICENSE) and also offered under commercial licenses, so
every contributor signs the [Contributor License Agreement](CLA.md) once before a pull request can be
merged. You keep the copyright in your work; the agreement lets it be included under both licenses.

When you open your first pull request, a bot comments with a link to the agreement. To sign, reply on
the pull request with:

```
I have read the CLA Document and I hereby sign the CLA
```

The check then passes, and your signature covers all your future pull requests. If it does not
update, comment `recheck`.

## Before opening a pull request

```bash
cargo test --release -p engine-sim   # the physics
npm test                             # the interface, and the Wasm build against the reference renders
```

After changing anything in Rust, rebuild the Wasm with `npm run build:sim`.
