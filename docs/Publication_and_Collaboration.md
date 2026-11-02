# Publication, opening and closing, and joint development

## One canonical website

All future simulation changes belong to this website project. The accepted version-3 folder is an archive. Edit src/ and docs/, run the scientific checks, build dist/, and publish a new version of the same Site. Reusing its stored project identity keeps the same public address.

The website is browser-local computation: visitors can change parameters and run simulations without uploading research data. Sharing an experiment-settings URL reproduces its initial configuration, not a shared live simulation session.

## Public, private, and closed access

Use the hosting platform's Site access policy. Public access allows anonymous visitors with the URL. To close visitor access while retaining owner access, switch to a custom owner-only policy and remove non-owner visitor/group grants. Reopening returns the policy to public. These are server-enforced permissions; a JavaScript switch or a hidden button is not an access-control mechanism.

The owner can request access-policy changes in the same development chat or use the platform's supported Site access controls. Only an authorized owner/editor can apply them. This public page intentionally contains no access token and no button that pretends to change privacy without backend authority.

Closing the Site does not delete the source, change its identity, or discard the published version. Previously downloaded browser assets are outside the scope of future request authorization. No recurring opening/closing schedule has been created. A schedule can be configured later after the owner specifies exact times and the desired audience.

## Search engines

The public build includes descriptive titles, canonical URLs, indexable explanatory pages, robots.txt, and sitemap.xml. A public URL can be shared immediately after successful deployment. Search-engine discovery and ranking are separate asynchronous processes; indexing is not guaranteed or immediate. A sitemap helps discovery but is not a guarantee.

Authentication and access policies protect closed content. robots.txt and noindex are search-engine directives, not security controls. The owner can optionally verify the domain in a search console and submit the sitemap. A custom domain can be attached after the owner supplies a domain they control and its DNS can be configured.

## Viewing is not developing

An anonymous visitor can operate the published simulator but cannot modify source code or publish a new version. A collaborator needs separate source-editing authority. Publication authority should remain with the owner unless the owner deliberately grants it.

For a collaborator in the same supported Sites workspace, native editor access can be granted to their account, separately from viewer access. An external email invitation may grant viewing only; it must not be described as a source-editor grant without checking its role.

For GitHub collaboration, use a private repository containing this project's source. A practical workflow is feature branch -> pull request -> scientific checks -> owner review -> accepted main branch -> publication to this existing Site. The repository may remain private while the website is public.

The source already contains a validation workflow and pull-request template. It does not contain deployment credentials, auto-invite an unknown person, or automatically deploy every collaborator push. The Code Owners file is a commented template until actual account names are confirmed.

Actual invitation requires the collaborator's GitHub username or authorized account identity and the owner's decision about publication authority. Do not share a personal account password, source token, or Site bearer token.

## Repository rules and plan limits

GitHub supports repository invitations and pull requests. Enforced branch protection and deployment environment protection depend on repository visibility and the GitHub plan. A private repository on a free plan does not automatically have every protected-branch feature. Until a rule is verified as active, owner review is a documented workflow rather than an enforced permission guarantee.

This project uses a hosted source repository for the current Site. A separate GitHub remote must be explicitly selected and connected before claiming that GitHub collaboration or branch protection is configured. The owner can provide an existing repository URL, or authorize creation/import into a new private repository. Contributor identity must be known before sending invitations.

## Development and publication sequence

```text
npm test
npm run build
npm run preview
```

On a bundled Node runtime without npm on PATH, use node tests/run-tests.cjs,
node scripts/build.mjs, and node scripts/serve.mjs instead.

The project has no production package dependency to install for core tests or its static build. The local browser regression uses the available Playwright runtime and Chrome. The hosted publication uses the supported source/version/deployment workflow and the persisted Site identity.

The validation workflow produces a reviewed static build on GitHub, but does not itself authorize production deployment. A failed source push is reconciled without a force push. A saved version provides a rollback point. Preserve the public audience on routine updates; changing visibility is a separate owner-requested action.

## References

- [GitHub: inviting repository collaborators](https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/repository-access-and-collaboration/inviting-collaborators-to-a-personal-repository)
- [GitHub: protected branches and eligible plans](https://docs.github.com/en/repositories/configuring-branches-and-merges-in-your-repository/managing-protected-branches)
- [Google Search technical requirements](https://developers.google.com/search/docs/essentials/technical)
- [Google: robots.txt is not access control](https://developers.google.com/search/docs/crawling-indexing/robots/intro)
